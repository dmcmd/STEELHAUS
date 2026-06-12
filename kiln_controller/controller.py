"""
controller.py — Main PID control loop for the kiln controller.

Key features:
  - Multi-zone PID: gains are looked up and interpolated by current temperature
    across all stored autotune zones, so behavior is optimal at any temperature.
  - Per-zone autotune: you can run autotune at any target temperature and the
    result is stored as a new zone without disturbing other zones.
  - Contactor relay management: contactor energizes on heater enable and drops
    on any fault, program complete, E-stop, or communication loss.
"""

import time
import threading
import logging
from typing import Optional

from pid_controller import PIDController, PIDParams, RelayAutotuner, RampRateController, RampStepAutotuner
from program_engine import ProgramRunner, ProgramRunState, Program, SegmentType
from esp32_bridge import ESP32Bridge, decode_fault
from program_store import (
    get_db, init_db,
    get_pid_params_for_temp, save_pid_zone, get_all_pid_zones,
    get_approach_params_for_temp,
    get_door_recovery_params,
    get_ramp_tune_params, save_ramp_tune_zone, get_all_ramp_tune_zones,
    update_thermal_model, get_hold_estimate, get_all_thermal_model,
    log_temperature, get_setting, set_setting, get_max_temp_c
)

logger = logging.getLogger("controller")

CONTROL_INTERVAL = 1.0   # seconds between PID calculations
LOG_INTERVAL     = 2.0   # seconds between temperature log writes
MODEL_UPDATE_INTERVAL_S = 30   # seconds between thermal model updates


class KilnController:
    """
    Central controller. Thread-safe. All public methods called by Flask API handlers.
    """

    def __init__(self, db_path: str, serial_port: str = "/dev/ttyUSB0"):
        self.db_path = db_path
        self._db = get_db(db_path)
        init_db(db_path)

        # Load initial PID params from zone table (uses interpolation at room temp)
        initial_params = get_pid_params_for_temp(self._db, 20.0)
        self._pid = PIDController(initial_params)

        self._runner  = ProgramRunner()
        self._bridge  = ESP32Bridge(
            port=serial_port,
            on_temp_update=self._on_temp_received,
            on_fault=self._on_fault,
        )

        self._lock    = threading.RLock()   # reentrant — callbacks from runner.tick() need this
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # Live state
        self._setpoint:      float = 20.0
        self._manual_mode:   bool  = True
        self._manual_output: Optional[float] = None
        self._pid_output:    float = 0.0
        self._current_temp:  float = 20.0
        self._last_log_time: float = 0.0
        self._last_zone_update_temp: float = -999.0   # track when to refresh PID zone

        # Autotune state
        self._autotuning:           bool  = False
        self._autotuner:            Optional[RelayAutotuner] = None
        self._autotune_result              = None
        self._autotune_method:      str   = "tyreus-luyben"
        self._autotune_setpoint:    float = 0.0
        self._autotune_preheat_stage: int = 0   # 0=idle, 1=full power, 2=cruise, 3=relay

        # Door state — open detection, integral freeze, and recovery mode
        self._door_was_open:        bool  = False
        self._door_open_count:      int   = 0
        self._door_contactor_retry: int   = 0   # Ticks to retry contactor after close
        self._door_open_start:      float = 0.0  # time.time() when door opened
        self._frozen_integral:      float = 0.0  # PID integral saved at door-open edge
        # Recovery mode — active after door closes
        self._recovery_active:      bool  = False
        self._recovery_boost_pct:   float = 100.0
        self._recovery_resume_offset_c: float = 5.6  # hand off to PID when within this many °C

        # Approach control phase tracking for UI display
        self._approach_phase:       str  = "pid"   # full | ramp | cruise | pid
        self._approach_pid_engaged: bool = False   # Latches True once PID takes over;
                                                   # resets on setpoint change or heater off

        # Ramp rate controller — used during program RAMP segments instead of PID
        self._ramp_rate_ctrl: Optional[RampRateController] = None
        self._ramp_rate_target: float = 0.0   # current target rate in °C/min

        # Ramp preheat: hold the program runner paused (don't call runner.start())
        # until the oven is actually climbing, so the ramp setpoint line starts
        # only when the oven is already moving.
        self._ramp_preheat_active: bool  = False
        self._ramp_preheat_rate_c: float = 0.0   # target rate (°C/min), used as threshold

        # Thermal model: steady-state output tracking
        # Updated every MODEL_UPDATE_INTERVAL_S when oven is genuinely settled.
        self._model_last_update:   float = 0.0
        self._model_stable_since:  float = 0.0   # when current stable window started
        self._model_last_output:   float = 0.0   # output on previous tick (for stability check)

        # Ramp step autotune state
        self._ramp_autotuning:  bool = False
        self._ramp_autotuner:   Optional[RampStepAutotuner] = None
        self._ramp_autotune_result: Optional[dict] = None
        self._ramp_autotune_temp_c: float = 0.0   # temp at which test was run

        # Safety
        self._heater_enabled: bool  = False
        self._max_temp_c:     float = get_max_temp_c(self._db)
        self._temp_offset_c:  float = float(get_setting(self._db, "temp_offset_c") or "0.0")
        self._session_id:     int   = 1   # Updated by app.py when a new session starts
        self._last_temp_for_log: float = 20.0  # Latest temp readable outside the lock
        self._last_door_for_log: bool  = False  # Latest door state readable outside the lock
        # Feature toggles — reloaded from DB when settings are saved
        self._approach_control_enabled: bool  = bool(int(get_setting(self._db, "approach_control_enabled") or "1"))
        self._door_recovery_enabled:    bool  = bool(int(get_setting(self._db, "door_recovery_enabled") or "1"))
        self._cruise_multiplier:        float = float(get_setting(self._db, "autotune_cruise_multiplier") or "1.0")
        self._relay_multiplier:         float = float(get_setting(self._db, "autotune_relay_multiplier") or "1.0")

    def start(self):
        self._bridge.start()
        self._running = True
        self._thread = threading.Thread(target=self._control_loop, daemon=True, name="control-loop")
        self._thread.start()
        logger.info("Kiln controller started")

    def stop(self):
        self._running = False
        self._heater_enabled = False
        self._bridge.set_output(duty=0, mosfet=0)
        self._bridge.set_contactor(False)
        logger.info("Kiln controller stopped")

    # ── Public API ────────────────────────────────────────────────────────────

    def set_session_id(self, session_id: int):
        """Called by app.py whenever a new session is created."""
        with self._lock:
            self._session_id = session_id
        logger.info(f"Controller session_id updated to {session_id}")

    def enable_heater(self, enable: bool):
        with self._lock:
            self._heater_enabled = enable
            if not enable:
                self._pid.reset()
                self._approach_pid_engaged = False
                self._approach_phase = "pid"
                self._bridge.set_output(duty=0, mosfet=0)
                self._bridge.set_contactor(False)
            else:
                # Contactor command is sent first — before any other work —
                # so nothing can interfere with it reaching the ESP32.
                self._bridge.set_contactor(True)
                # Seed PID integral from thermal model. Wrapped in try/except
                # so a DB error can never prevent the oven from starting.
                try:
                    model_hold = get_hold_estimate(self._db, self._setpoint)
                    if model_hold is not None and model_hold > 0:
                        self._pid._integral = model_hold
                        logger.info(
                            f"Heater enabled — PID integral seeded at {model_hold:.1f}% "
                            f"from thermal model at {self._setpoint:.0f}°C"
                        )
                except Exception as e:
                    logger.warning(f"Heater enable: thermal model seed failed ({e}) — continuing")
        logger.info(f"Heater {'enabled — contactor energized' if enable else 'disabled — contactor dropped'}")

    def set_manual_setpoint(self, temp_c: float):
        with self._lock:
            self._setpoint = float(temp_c)
            self._manual_mode = True
            self._approach_pid_engaged = False  # New setpoint restarts approach sequence
            if self._runner.state == ProgramRunState.RUNNING:
                self._runner.abort()
            # Seed PID integral from thermal model for this setpoint.
            # Wrapped in try/except so a DB error never breaks setpoint changes.
            try:
                model_hold = get_hold_estimate(self._db, float(temp_c))
                if model_hold is not None and model_hold > 0:
                    self._pid._integral = model_hold
                    logger.info(
                        f"Manual setpoint {temp_c:.0f}°C — PID integral pre-seeded "
                        f"at {model_hold:.1f}% from thermal model"
                    )
            except Exception as e:
                logger.warning(f"set_manual_setpoint: thermal model seed failed ({e})")

    def set_manual_output(self, pct: Optional[float]):
        with self._lock:
            self._manual_output = pct

    def load_program(self, program: Program):
        with self._lock:
            self._runner.load(
                program, self._current_temp,
                on_complete=self._on_program_complete,
                on_segment_change=self._on_segment_change,
            )
            self._manual_mode = False

    def start_program(self):
        with self._lock:
            if not self._heater_enabled:
                self._heater_enabled = True
                self._bridge.set_contactor(True)
            self._manual_mode = False
            segs = self._runner._program.segments if self._runner._program else []
            if segs and segs[0].type == SegmentType.RAMP:
                rate_c = segs[0].rate_per_min
                gains  = get_ramp_tune_params(self._db, self._current_temp)
                rrc = RampRateController(
                    target_rate_per_min=rate_c,
                    kp=gains["kp"], ki=gains["ki"],
                    K_proc=gains.get("K_proc", 0.0),
                )
                # Seed integral from thermal model so rate controller starts
                # at the right baseline rather than building from zero.
                model_hold = get_hold_estimate(self._db, self._current_temp)
                if model_hold is not None and model_hold > 0:
                    rrc._integral = model_hold
                    logger.info(
                        f"Ramp rate ctrl: integral seeded at {model_hold:.1f}% "
                        f"from thermal model at {self._current_temp:.0f}°C"
                    )
                self._ramp_rate_ctrl      = rrc
                self._ramp_rate_target    = rate_c
                # Preheat: heat with approach+PID but hold the runner clock until
                # the oven is actually climbing (≥20% of target rate).
                self._ramp_preheat_active = True
                self._ramp_preheat_rate_c = rate_c
                self._setpoint = self._current_temp   # approach heats from here
                logger.info(
                    f"Program queued: first segment RAMP at {rate_c:.2f}°C/min "
                    f"— preheat active, waiting for oven to start climbing"
                )
            else:
                # Soak first — start runner immediately
                self._runner.start(self._current_temp)
                self._ramp_preheat_active  = False
                self._ramp_rate_ctrl       = None
                self._approach_pid_engaged = False
                self._pid.reset()

    def pause_program(self):
        with self._lock:
            self._runner.pause()

    def resume_program(self):
        with self._lock:
            self._runner.resume()

    def abort_program(self):
        with self._lock:
            self._runner.abort()
            self._manual_mode         = True
            self._ramp_preheat_active = False
            self._ramp_rate_ctrl      = None

    # ── Autotune helpers ──────────────────────────────────────────────────────

    @staticmethod
    def _autotune_loss_rate_c_per_min(temp_c: float) -> float:
        """
        Estimated oven heat-loss rate at a given temperature.
        Power law fit from three measured data points:
          420°F (215°C): 8.5°F/min (4.7°C/min)
          782°F (417°C): 36°F/min  (20.0°C/min)
          875°F (468°C): 47°F/min  (26.1°C/min)
        Fit: loss = 0.000097 × (T - T_ambient)^2.047  [°C/min]
        Radiation + convection combined loss scales close to T² over this range.
        """
        ambient_c = 21.0
        delta = max(0.0, temp_c - ambient_c)
        return 0.000097 * (delta ** 2.047)

    @staticmethod
    def _autotune_cruise_output(temp_c: float) -> float:
        """
        Output % needed to climb at ~5.5°C/min (10°F/min) against heat loss.
        Heating rate: 100% ≈ 222°C/min  →  1% ≈ 2.22°C/min
        cruise_output = (loss_rate + 5.5) / 2.22
        Capped at 75%. Multiplier is applied on top via _autotune_cruise_output_scaled().
        """
        loss = KilnController._autotune_loss_rate_c_per_min(temp_c)
        return min((loss + 5.5) / 2.22, 75.0)

    def _autotune_cruise_output_scaled(self, temp_c: float) -> float:
        """Cruise output for autotune preheat stage 2.

        When the thermal model has data near this temperature, use it as the
        primary source — hold_power + a small climb margin gets us to setpoint
        cleanly without overshoot.  Fall back to the formula if no model data.
        """
        formula_output = min(self._autotune_cruise_output(temp_c) * self._cruise_multiplier, 100.0)
        model_hold = get_hold_estimate(self._db, temp_c)
        if model_hold is not None and model_hold > 0:
            # Climb at hold + 8% so we approach setpoint gradually rather than fast
            model_cruise = min(model_hold + 8.0, 100.0) * self._cruise_multiplier
            # Blend: 70% model, 30% formula — model wins but formula guards against
            # bad data (e.g. very first run before model has enough points)
            blended = 0.7 * model_cruise + 0.3 * formula_output
            logger.debug(
                f"Autotune cruise at {temp_c:.0f}°C: model={model_hold:.1f}%+8 "
                f"formula={formula_output:.1f}% → blended={blended:.1f}%"
            )
            return min(blended, 100.0)
        return formula_output

    def _autotune_relay_high_scaled(self, temp_c: float) -> float:
        """
        Relay output high = 2.5× base cruise output × relay multiplier, capped at 100%.
        Uses its own independent multiplier so cruise preheat power and relay
        oscillation amplitude can be tuned separately.
        """
        base_relay = min(self._autotune_cruise_output(temp_c) * 2.5, 100.0)
        return min(base_relay * self._relay_multiplier, 100.0)

    @staticmethod
    def _autotune_preheat_thresholds_c(setpoint_c: float):
        """
        Return (stage1_delta_c, stage2_delta_c) — the °C offsets below setpoint
        where stage 1 (full power) ends and stage 2 (cruise) ends respectively.

        Thresholds scale inversely with setpoint so the cruise phase stays short
        at high temperatures where heat loss is large and cruise output is high.
        At low temps: 150°F / 60°F below setpoint (same as original).
        At high temps: shrinks to 50°F / 20°F minimum.

        Formula: threshold = max(min_f, base_f × (400 / setpoint_f)) → convert to °C delta
        """
        setpoint_f = setpoint_c * 9/5 + 32
        stage1_f = max(50.0, 150.0 * (400.0 / setpoint_f))
        stage2_f = max(20.0,  60.0 * (400.0 / setpoint_f))
        return stage1_f * 5/9, stage2_f * 5/9   # °F delta → °C delta

    def start_autotune(self, setpoint_c: float, method: str = "some-overshoot"):
        """
        Start relay-feedback autotune at the given setpoint.

        Stage 1 — full power (100%) until stage1_threshold below setpoint.
        Stage 2 — scaled cruise output until stage2_threshold below setpoint.
        Stage 3 — relay oscillation. relay_output_high is read live from
                  _autotune_relay_high_scaled() each tick so multiplier
                  changes take effect immediately without restarting.

        When complete, the result is stored as a PID zone for that temperature.
        """
        s1_c, s2_c = self._autotune_preheat_thresholds_c(setpoint_c)
        relay_high  = self._autotune_relay_high_scaled(setpoint_c)
        with self._lock:
            if not self._heater_enabled:
                self._heater_enabled = True
                self._bridge.set_contactor(True)
            self._autotuning             = True
            self._autotune_method        = method
            self._autotune_setpoint      = setpoint_c
            self._autotune_preheat_stage = 1
            # relay_output_high is set to 0 here — it is updated live each tick
            # in the control loop so multiplier changes take effect immediately
            self._autotuner = RelayAutotuner(
                setpoint=setpoint_c,
                relay_output_high=relay_high,
                relay_output_low=0.0,
                noise_band=2.0,
                required_cycles=5,
            )
            self._autotune_result = None
        logger.info(
            f"Autotune started: setpoint={setpoint_c:.0f}°C, method={method}\n"
            f"  Stage 1: 100% until {setpoint_c - s1_c:.0f}°C "
            f"({(setpoint_c - s1_c)*9/5+32:.0f}°F)\n"
            f"  Stage 2: {self._autotune_cruise_output_scaled(setpoint_c):.1f}% "
            f"(base={self._autotune_cruise_output(setpoint_c):.1f}%, "
            f"mult={self._cruise_multiplier:.2f}×) "
            f"until {setpoint_c - s2_c:.0f}°C ({(setpoint_c - s2_c)*9/5+32:.0f}°F)\n"
            f"  Stage 3: relay high={relay_high:.1f}% "
            f"(base={min(self._autotune_cruise_output(setpoint_c)*2.5,100):.1f}%, "
            f"mult={self._relay_multiplier:.2f}×)"
        )

    def abort_autotune(self):
        with self._lock:
            self._autotuning             = False
            self._autotune_preheat_stage = 0
            self._autotuner              = None
            self._heater_enabled         = False
            self._bridge.set_output(duty=0, mosfet=0)
            self._bridge.set_contactor(False)

    def apply_autotune_result(self):
        """
        Save the most recent autotune result as a PID zone.
        Does not overwrite other zones.
        """
        with self._lock:
            if self._autotune_result is None:
                raise RuntimeError("No autotune result available")
            r = self._autotune_result
            save_pid_zone(
                self._db,
                center_temp_c=self._autotune_setpoint,
                kp=r.kp, ki=r.ki, kd=r.kd,
                ku=r.ku, tu=r.tu,
                method=r.method,
            )
            # Immediately apply to running PID
            self._pid.set_params(PIDParams(
                kp=r.kp, ki=r.ki, kd=r.kd,
                output_min=0, output_max=100, sample_time=1.0
            ))
            logger.info(f"Autotune result saved as zone at {self._autotune_setpoint:.0f}°C")

    def delete_pid_zone(self, zone_id: int):
        from program_store import delete_pid_zone as _delete
        _delete(self._db, zone_id)

    def set_pid_params_manual(self, kp: float, ki: float, kd: float):
        """Override PID params directly (not zone-based). Useful for manual fine-tuning."""
        with self._lock:
            params = PIDParams(kp=kp, ki=ki, kd=kd, output_min=0, output_max=100, sample_time=1.0)
            self._pid.set_params(params)

    def emergency_stop(self):
        with self._lock:
            self._heater_enabled = False
            self._autotuning     = False
            if self._runner.state == ProgramRunState.RUNNING:
                self._runner.abort()
        self._bridge.emergency_stop()
        self._bridge.set_contactor(False)
        logger.warning("EMERGENCY STOP triggered")

    def reset_estop(self):
        self._bridge.reset()
        # Send immediate ping so watchdog clears right away
        self._bridge.ping()
        logger.info("E-Stop reset")

    # ── Ramp Step Autotune ────────────────────────────────────────────────────

    def start_ramp_autotune(self, stabilize_temp_c: float, step_output: float = 50.0,
                            step_duration_s: float = 360.0):
        """
        Start a ramp step-response autotune.
        The oven stabilizes at stabilize_temp_c, then a fixed step_output is
        applied for step_duration_s seconds.  Gains are derived automatically.
        """
        with self._lock:
            self._ramp_autotune_result = None
            self._ramp_autotune_temp_c = stabilize_temp_c
            self._ramp_autotuner = RampStepAutotuner(
                stabilize_setpoint_c=stabilize_temp_c,
                step_output=step_output,
                step_duration_s=step_duration_s,
            )
            self._ramp_autotuner.start()
            self._ramp_autotuning = True
            if not self._heater_enabled:
                self._heater_enabled = True
                self._bridge.set_contactor(True)
        logger.info(
            f"Ramp autotune started: stabilize={stabilize_temp_c:.1f}°C, "
            f"step={step_output}%, duration={step_duration_s}s"
        )

    def abort_ramp_autotune(self):
        with self._lock:
            self._ramp_autotuning = False
            if self._ramp_autotuner:
                self._ramp_autotuner.abort()
            self._heater_enabled = False
            self._bridge.set_output(duty=0, mosfet=0)
            self._bridge.set_contactor(False)
        logger.info("Ramp autotune aborted")

    def apply_ramp_autotune_result(self):
        """Save the ramp autotune result as a zone in the database."""
        with self._lock:
            r       = self._ramp_autotune_result
            temp_c  = self._ramp_autotune_temp_c
            # Capture hold_pct from the autotuner before it's garbage collected
            hold    = self._ramp_autotuner.hold_pct if self._ramp_autotuner else 0.0
        if r is None:
            raise RuntimeError("No ramp autotune result to save")
        save_ramp_tune_zone(
            self._db,
            center_temp_c=temp_c,
            kp=r["kp"], ki=r["ki"],
            K_proc=r.get("K", 0.0),
            L_s=r.get("L", 0.0),
            step_output=r.get("step_output", 50.0),
            hold_pct=hold,
        )
        logger.info(
            f"Ramp tune zone saved at {temp_c:.1f}°C: "
            f"kp={r['kp']}, ki={r['ki']}, hold_pct={hold:.1f}%"
        )

    def delete_ramp_tune_zone(self, zone_id: int):
        from program_store import delete_ramp_tune_zone as _del
        _del(self._db, zone_id)

    def reload_max_temp(self):
        """Call after user updates max temp setting."""
        self._max_temp_c = get_max_temp_c(self._db)
        logger.info(f"Max temp updated to {self._max_temp_c:.0f}°C")

    def reload_temp_offset(self):
        """Call after user updates temp offset setting."""
        self._temp_offset_c = float(get_setting(self._db, "temp_offset_c") or "0.0")
        logger.info(f"Temp offset updated to {self._temp_offset_c:.2f}°C")

    def reload_approach_control(self, val: bool = None):
        """Call after user toggles approach control on/off."""
        if val is None:
            val = bool(int(get_setting(self._db, "approach_control_enabled") or "1"))
        with self._lock:
            self._approach_control_enabled = val
        logger.info(f"Approach control {'enabled' if val else 'disabled'}")

    def reload_door_recovery(self, val: bool = None):
        """Call after user toggles door recovery on/off."""
        if val is None:
            val = bool(int(get_setting(self._db, "door_recovery_enabled") or "1"))
        with self._lock:
            self._door_recovery_enabled = val
        logger.info(f"Door recovery {'enabled' if val else 'disabled'}")

    def reload_cruise_multiplier(self, val: float = None):
        """Call after user updates the autotune cruise multiplier."""
        if val is None:
            val = max(0.1, float(get_setting(self._db, "autotune_cruise_multiplier") or "1.0"))
        with self._lock:
            self._cruise_multiplier = val
        logger.info(f"Autotune cruise multiplier updated to {val:.2f}")

    def reload_relay_multiplier(self, val: float = None):
        """Call after user updates the autotune relay high multiplier."""
        if val is None:
            val = max(0.1, float(get_setting(self._db, "autotune_relay_multiplier") or "1.0"))
        with self._lock:
            self._relay_multiplier = val
        logger.info(f"Autotune relay multiplier updated to {val:.2f}")

    def get_status(self) -> dict:
        with self._lock:
            esp  = self._bridge.state
            prms = self._pid.params
            rs   = self._runner.get_status(self._current_temp, self._pid_output)
            at_result = None
            if self._autotune_result:
                r = self._autotune_result
                at_result = {"kp": r.kp, "ki": r.ki, "kd": r.kd,
                             "ku": r.ku, "tu": r.tu, "method": r.method}
            ramp_at_phase = (
                self._ramp_autotuner.phase if self._ramp_autotuner else "idle"
            )
            ramp_at_elapsed = (
                self._ramp_autotuner.elapsed_step_s if self._ramp_autotuner else 0.0
            )

        zones      = get_all_pid_zones(self._db)
        ramp_zones = get_all_ramp_tune_zones(self._db)

        return {
            "temp":           self._current_temp,
            "cold_junction":  esp.cold_junction,
            "setpoint":       self._setpoint,
            "pid_output":      self._pid_output,
            "approach_phase":  self._approach_phase,
            "heater_enabled":  self._heater_enabled,
            "manual_mode":     self._manual_mode,
            "manual_output":  self._manual_output,
            "ssr_duty":       esp.ssr_duty,
            "contactor":      esp.contactor,
            "mosfet":         esp.mosfet_duty,
            "sim_mode":       False,
            "fault":          esp.fault_code,
            "fault_messages": decode_fault(esp.fault_code),
            "estop":          esp.estop,
            "watchdog":        esp.watchdog_tripped,
            "door_open":       esp.door_open,
            "door_recovery_active":     self._recovery_active,
            "approach_control_enabled": self._approach_control_enabled,
            "door_recovery_enabled":    self._door_recovery_enabled,
            "cruise_multiplier":        self._cruise_multiplier,
            "relay_multiplier":         self._relay_multiplier,
            "esp32_connected": esp.connected,
            "autotuning":              self._autotuning,
            "autotune_preheat_stage":  self._autotune_preheat_stage,
            "autotune_done":           self._autotuner.done        if self._autotuner else False,
            "autotune_cycles":         self._autotuner.cycle_count if self._autotuner else 0,
            "autotune_peak_high":      self._autotuner.peak_high   if self._autotuner else None,
            "autotune_peak_low":       self._autotuner.peak_low    if self._autotuner else None,
            "autotune_result": at_result,
            "autotune_setpoint": self._autotune_setpoint,
            "pid": {"kp": prms.kp, "ki": prms.ki, "kd": prms.kd},
            "pid_zones": zones,
            "ramp_autotuning":       self._ramp_autotuning,
            "ramp_autotune_phase":   ramp_at_phase,
            "ramp_autotune_elapsed": round(ramp_at_elapsed, 0),
            "ramp_autotune_done":    self._ramp_autotune_result is not None,
            "ramp_autotune_result":  self._ramp_autotune_result,
            "ramp_autotune_temp_c":  self._ramp_autotune_temp_c,
            "ramp_tune_zones":       ramp_zones,
            "max_temp_c": self._max_temp_c,
            "temp_offset_c": self._temp_offset_c,
            "program": {
                "state":               rs.state.value,
                "id":                  self._runner._program.id if self._runner._program else None,
                "name":                rs.program_name,
                "segment_index":       rs.segment_index,
                "segment_total":       rs.segment_total,
                "segment_type":        rs.segment_type,
                "segment_progress_pct": rs.segment_progress_pct,
                "overall_progress_pct": rs.overall_progress_pct,
                "elapsed_min":         rs.elapsed_min,
                "remaining_min":       rs.remaining_min,
            },
        }

    # ── Internal control loop ─────────────────────────────────────────────────

    def _control_loop(self):
        last_control = time.time()

        while self._running:
            now = time.time()
            if (now - last_control) < CONTROL_INTERVAL:
                time.sleep(0.05)
                continue
            last_control = now

            with self._lock:
                current_temp = self._bridge.temperature + self._temp_offset_c
                self._current_temp = current_temp
                door_open = self._bridge.state.door_open
                # Store on self so the log block outside the lock can always read them
                self._last_temp_for_log = current_temp
                self._last_door_for_log = door_open

                if not self._heater_enabled:
                    self._pid_output = 0.0
                    self._bridge.set_output(duty=0, mosfet=0)
                    continue  # Restored: skip PID/autotune — log block below handles logging

                # Over-temp safety
                if current_temp > self._max_temp_c:
                    logger.error(f"OVER-TEMP: {current_temp:.1f}°C > {self._max_temp_c:.1f}°C — shutting down")
                    self._heater_enabled = False
                    self._bridge.set_output(duty=0, mosfet=0)
                    self._bridge.set_contactor(False)
                    continue  # Skip PID; log block will capture this via _last_temp_for_log

                # Update PID zone based on current temperature.
                # Re-evaluate every time temp moves more than 25°C from last update.
                if abs(current_temp - self._last_zone_update_temp) > 25.0:
                    new_params = get_pid_params_for_temp(self._db, current_temp)
                    self._pid.set_params(new_params)
                    self._last_zone_update_temp = current_temp

                # ── Door state tracking ──────────────────────────────────────
                if door_open:
                    self._door_open_count += 1
                    if not self._door_was_open:
                        # Door just opened — freeze the PID integral so it doesn't
                        # wind up during the open period. We'll restore it on close.
                        self._frozen_integral = self._pid._integral
                        self._door_open_start = now
                        logger.info(f"Door opened — integral frozen at {self._frozen_integral:.3f}")
                    # While door is open: hold integral frozen, don't update PID.
                    # But keep _last_time and _last_measurement current so the first
                    # post-recovery PID tick sees dt=1s, not however long the door was open.
                    self._pid._integral        = self._frozen_integral
                    self._pid._last_time       = now
                    self._pid._last_measurement = current_temp
                else:
                    if self._door_was_open and self._door_open_count >= 1:
                        # Door just closed — start recovery
                        open_duration = now - self._door_open_start
                        rp = get_door_recovery_params(self._db, self._setpoint) \
                             if self._door_recovery_enabled else None
                        if rp is not None:
                            self._pid._integral = self._frozen_integral
                            self._recovery_boost_pct          = rp["boost_pct"]
                            self._recovery_resume_offset_c    = rp["resume_offset_c"]
                            self._recovery_active             = True
                            logger.info(
                                f"Door closed after {open_duration:.1f}s — recovery: "
                                f"{rp['boost_pct']:.0f}% until within "
                                f"{rp['resume_offset_c']:.1f}°C of setpoint"
                            )
                        else:
                            # No recovery zone — seed PID from thermal model
                            model_hold = get_hold_estimate(self._db, self._setpoint)
                            steady_state = model_hold if (model_hold and model_hold > 0) \
                                           else self._frozen_integral
                            self._pid._integral         = steady_state
                            self._pid._last_time        = now
                            self._pid._last_measurement = current_temp
                            logger.info(
                                f"Door closed — no recovery zone, PID seeded at "
                                f"{steady_state:.1f}% from thermal model"
                            )
                        self._door_contactor_retry = 5
                    self._door_open_count = 0
                    # Retry contactor for several ticks after close in case ESP32
                    # refused it while door was still registering as open
                    if self._door_contactor_retry > 0:
                        self._bridge.set_contactor(True)
                        self._door_contactor_retry -= 1
                self._door_was_open = door_open

                # Ramp step autotune mode
                _rat_step_active = False
                if self._ramp_autotuning and self._ramp_autotuner:
                    rat = self._ramp_autotuner

                    if rat.phase == "stabilize":
                        # Passive stabilize — normal approach+PID heats to setpoint.
                        # Override the setpoint so the full approach control chain
                        # (approach zones, cruise, PID hand-off) works exactly as
                        # it does during manual operation.
                        self._setpoint = rat.stabilize_setpoint
                        rat.monitor_stabilize(current_temp, now,
                                              pid_output=self._pid_output)
                        if rat.ready_to_step(now):
                            rat.begin_step(now)
                            # Latch PID engaged so approach control doesn't restart
                            self._approach_pid_engaged = True

                    if rat.phase == "step":
                        _rat_step_active = True
                        output = rat.update_step(current_temp, now)
                        self._pid_output = output
                        self._bridge.set_output(duty=int(round(output)), mosfet=0)
                        if rat.done:
                            self._ramp_autotune_result = rat.result
                            if self._ramp_autotune_result:
                                self._ramp_autotune_result["step_output"] = rat.step_output
                            self._ramp_autotuning      = False
                            self._heater_enabled       = False
                            self._approach_pid_engaged = False
                            self._bridge.set_output(duty=0, mosfet=0)
                            self._bridge.set_contactor(False)
                            logger.info(f"Ramp autotune complete: {self._ramp_autotune_result}")

                if _rat_step_active:
                    pass   # output already set above; skip PID/autotune blocks

                # Regular PID autotune mode
                elif self._autotuning and self._autotuner:
                    sp  = self._autotune_setpoint
                    stg = self._autotune_preheat_stage
                    s1_c, s2_c = self._autotune_preheat_thresholds_c(sp)

                    if stg == 1:
                        # Stage 1: full power until s1_c below setpoint
                        if current_temp >= sp - s1_c:
                            self._autotune_preheat_stage = 2
                            logger.info(
                                f"Autotune: stage 2 — cruise at "
                                f"{self._autotune_cruise_output_scaled(sp):.1f}% until {sp - s2_c:.0f}°C"
                            )
                        output = 100.0

                    elif stg == 2:
                        # Stage 2: cruise until s2_c below setpoint
                        if current_temp >= sp - s2_c:
                            self._autotune_preheat_stage = 3
                            logger.info("Autotune: stage 3 — relay oscillation started")
                        output = self._autotune_cruise_output_scaled(sp)

                    else:
                        # Stage 3: relay oscillation
                        # Update relay_output_high live so multiplier changes
                        # take effect without needing to restart autotune
                        self._autotuner.relay_high = self._autotune_relay_high_scaled(sp)
                        output = self._autotuner.update(current_temp, now)

                    self._pid_output = output
                    self._bridge.set_output(duty=int(round(output)), mosfet=0)

                    if stg == 3 and self._autotuner.done:
                        self._autotune_result = self._autotuner.get_result(self._autotune_method)
                        self._autotuning            = False
                        self._autotune_preheat_stage = 0
                        self._heater_enabled        = False
                        self._bridge.set_output(duty=0, mosfet=0)
                        self._bridge.set_contactor(False)
                        logger.info(f"Autotune complete at {sp:.0f}°C: {self._autotune_result}")

                else:
                    # Normal PID / recovery mode

                    # ── Ramp preheat: hold runner clock until oven is climbing ─
                    # While _ramp_preheat_active, approach+PID heats the oven.
                    # Once actual rate ≥ 20% of target rate, release the runner.
                    if self._ramp_preheat_active and self._ramp_rate_ctrl is not None:
                        # Feed samples to the rate controller so actual_rate is ready
                        self._ramp_rate_ctrl._samples.append((now, current_temp))
                        cutoff = now - self._ramp_rate_ctrl.window_s
                        self._ramp_rate_ctrl._samples = [
                            (t, v) for t, v in self._ramp_rate_ctrl._samples if t >= cutoff
                        ]
                        actual_rate = self._ramp_rate_ctrl.actual_rate
                        threshold   = self._ramp_preheat_rate_c * 0.20
                        if actual_rate >= threshold:
                            self._ramp_preheat_active = False
                            self._runner.start(self._current_temp)
                            logger.info(
                                f"Preheat complete: oven at {actual_rate:.2f}°C/min "
                                f"(threshold {threshold:.2f}°C/min) — runner started"
                            )
                        # else: keep setpoint at current temp, approach heats the oven

                    if not self._manual_mode and self._runner.state == ProgramRunState.RUNNING:
                        self._setpoint = self._runner.tick(current_temp, now)

                    if self._manual_output is not None:
                        output = max(0.0, min(100.0, self._manual_output))
                        self._recovery_active = False

                    elif self._recovery_active and not door_open:
                        # ── Door recovery mode ────────────────────────────────
                        # Apply boost_pct until current_temp reaches
                        # setpoint - resume_offset_c, then hand to PID
                        # seeded from thermal model.
                        resume_threshold = self._setpoint - self._recovery_resume_offset_c
                        if current_temp >= resume_threshold:
                            # Temp is back — end recovery, seed PID from thermal model
                            model_hold = get_hold_estimate(self._db, self._setpoint)
                            steady_state = model_hold if (model_hold and model_hold > 0) \
                                           else self._frozen_integral
                            self._recovery_active      = False
                            self._approach_pid_engaged = True
                            self._pid.reset()
                            self._pid._integral         = steady_state
                            self._pid._last_time        = now
                            self._pid._last_measurement = current_temp
                            output = self._pid.update(self._setpoint, current_temp, now)
                            self._approach_phase = "pid"
                            logger.info(
                                f"Door recovery complete at {current_temp:.1f}°C — "
                                f"PID seeded at {steady_state:.1f}% from thermal model"
                            )
                        else:
                            # Still recovering — apply boost output
                            output = self._recovery_boost_pct
                            self._approach_phase = "door-recovery"
                            # Keep PID time tracking current for smooth handoff
                            self._pid._last_time        = now
                            self._pid._last_measurement = current_temp
                            output = self._pid.update(self._setpoint, current_temp, now)
                            self._approach_phase = "pid"
                            logger.info("Door recovery complete — PID resumed")

                    else:
                        if self._ramp_rate_ctrl is not None:
                            # ── Program RAMP segment — rate controller ────────
                            output = self._ramp_rate_ctrl.update(
                                current_temp, now,
                                setpoint=self._setpoint   # for position clamping
                            )
                            self._approach_phase = "ramp"
                            # Keep PID state current so hand-off to soak PID is smooth
                            self._pid._last_time = now
                            self._pid._last_measurement = current_temp
                            self._pid._integral = output

                        else:
                            # ── Normal operation — multi-phase approach + PID ─
                            ap = get_approach_params_for_temp(self._db, self._setpoint) if self._approach_control_enabled else None
                            error = self._setpoint - current_temp

                            if ap is None:
                                if error > 55.6 and not self._approach_pid_engaged:
                                    # Check thermal model — if we know hold power,
                                    # use it as a cruise target instead of full blast
                                    model_hold = get_hold_estimate(self._db, self._setpoint)
                                    if model_hold is not None and model_hold > 0 and error <= 111.0:
                                        # Within 200°F — cruise to setpoint
                                        output = min(model_hold + 10.0, 100.0)
                                        self._approach_phase = "cruise"
                                        self._pid.reset()
                                    else:
                                        output = 100.0
                                        self._pid.reset()
                                        self._approach_phase = "full"
                                else:
                                    if not self._approach_pid_engaged:
                                        # First tick into PID — seed integral from thermal model
                                        model_hold = get_hold_estimate(self._db, self._setpoint)
                                        if model_hold is not None and model_hold > 0:
                                            self._pid._integral = model_hold
                                            logger.info(
                                                f"Approach→PID handoff: integral seeded at "
                                                f"{model_hold:.1f}% from thermal model"
                                            )
                                    self._approach_pid_engaged = True
                                    output = self._pid.update(self._setpoint, current_temp, now)
                                    self._approach_phase = "pid"

                            else:
                                fp  = ap["full_power_offset_c"]
                                rs  = ap["ramp_start_offset_c"]
                                cp  = ap["cruise_power"]
                                po  = ap["pid_offset_c"]

                                # Blend thermal model into cruise power when available.
                                # The stored zone defines approach shape; the model
                                # refines the actual level from real oven behaviour.
                                model_hold = get_hold_estimate(self._db, self._setpoint)
                                if model_hold is not None and model_hold > 0:
                                    cp = 0.7 * (model_hold + 6.0) + 0.3 * cp
                                    cp = max(0.0, min(100.0, cp))

                                if self._approach_pid_engaged:
                                    output = self._pid.update(self._setpoint, current_temp, now)
                                    self._approach_phase = "pid"

                                elif error > fp:
                                    output = 100.0
                                    self._pid.reset()
                                    self._approach_phase = "full"

                                elif error > rs:
                                    ramp_span = fp - rs
                                    t = (error - rs) / ramp_span
                                    output = cp + t * (100.0 - cp)
                                    self._pid.reset()
                                    self._approach_phase = "ramp"

                                elif error > po:
                                    output = cp
                                    self._pid.reset()
                                    self._approach_phase = "cruise"

                                else:
                                    if not self._approach_pid_engaged:
                                        # First tick into PID — seed integral from thermal model
                                        model_hold = get_hold_estimate(self._db, self._setpoint)
                                        if model_hold is not None and model_hold > 0:
                                            self._pid._integral = model_hold
                                            logger.info(
                                                f"Approach→PID handoff: integral seeded at "
                                                f"{model_hold:.1f}% from thermal model"
                                            )
                                    self._approach_pid_engaged = True
                                    output = self._pid.update(self._setpoint, current_temp, now)
                                    self._approach_phase = "pid"

                    self._pid_output = output
                    self._bridge.set_output(duty=int(round(output)), mosfet=0)

            # Log at reduced rate — always runs regardless of heater state.
            # Uses _last_temp_for_log / _last_door_for_log set inside the lock above,
            # so it works even when the loop hit a continue before reaching here.
            if now - self._last_log_time >= LOG_INTERVAL:
                self._last_log_time = now
                try:
                    _t  = self._last_temp_for_log
                    _do = self._last_door_for_log
                    rs  = self._runner.get_status(_t, self._pid_output)
                    log_temperature(self._db, now, _t, self._setpoint,
                                    self._pid_output, rs.program_name, rs.segment_index,
                                    session_id=self._session_id, door_open=int(_do))
                except Exception as e:
                    logger.error(f"Log error: {e}")

            # ── Thermal model update ──────────────────────────────────────────
            # Record steady-state hold power whenever the oven is genuinely settled.
            # Conditions for a valid observation:
            #   - PID is in control (not approach, not ramp, not manual, not autotune)
            #   - Not in door recovery
            #   - Door is closed
            #   - Temperature within 3°C of setpoint (actually at setpoint)
            #   - Output has been stable (< 3% change) for at least 60 seconds
            try:
                _stable_cond = (
                    self._approach_phase == "pid"
                    and self._heater_enabled              # oven is actively controlled
                    and not self._autotuning              # not running relay autotune
                    and not self._ramp_autotuning         # not running ramp step test
                    and not self._recovery_active         # not in door recovery
                    and not self._ramp_preheat_active     # not in ramp preheat
                    and self._ramp_rate_ctrl is None      # not actively ramping
                    and not self._last_door_for_log       # door is closed
                    and abs(self._last_temp_for_log - self._setpoint) <= 3.0  # at setpoint
                )
                if _stable_cond:
                    output_drift = abs(self._pid_output - self._model_last_output)
                    if output_drift < 3.0:
                        # Output is stable — extend/start the stable window
                        if self._model_stable_since == 0.0:
                            self._model_stable_since = now
                    else:
                        # Output changed significantly — reset stable window
                        self._model_stable_since = 0.0

                    # Update model if we've been stable for 60s and enough time
                    # has passed since the last model write
                    if (self._model_stable_since > 0
                            and now - self._model_stable_since >= 60.0
                            and now - self._model_last_update >= MODEL_UPDATE_INTERVAL_S):
                        update_thermal_model(
                            self._db,
                            temp_c=self._setpoint,      # use setpoint as the key temp
                            hold_pct=self._pid_output,
                        )
                        self._model_last_update = now
                        logger.info(
                            f"Thermal model updated: {self._setpoint:.1f}°C "
                            f"→ {self._pid_output:.1f}%"
                        )
                else:
                    self._model_stable_since = 0.0

                self._model_last_output = self._pid_output
            except Exception as e:
                logger.error(f"Thermal model update error: {e}")

    def _on_temp_received(self, temp, ts):
        pass  # Handled via bridge.temperature in control loop

    def _on_fault(self, fault_code):
        logger.error(f"Thermocouple fault {fault_code:08b}: {decode_fault(fault_code)}")
        with self._lock:
            self._heater_enabled = False
        self._bridge.set_output(duty=0, mosfet=0)
        self._bridge.set_contactor(False)

    def _on_program_complete(self):
        logger.info("Program completed — contactor dropped")
        with self._lock:
            self._heater_enabled = False
            self._manual_mode    = True
            self._ramp_rate_ctrl = None
        self._bridge.set_output(duty=0, mosfet=0)
        self._bridge.set_contactor(False)

    def _on_segment_change(self, from_idx, to_idx):
        """Called by ProgramRunner when advancing to the next segment."""
        with self._lock:
            segs = self._runner._program.segments if self._runner._program else []
            if to_idx < len(segs):
                seg = segs[to_idx]
                if seg.type == SegmentType.RAMP:
                    rate_c = seg.rate_per_min   # °C/min (internal storage unit)
                    gains  = get_ramp_tune_params(self._db, self._current_temp)
                    rrc = RampRateController(
                        target_rate_per_min=rate_c,
                        kp=gains["kp"], ki=gains["ki"],
                        K_proc=gains.get("K_proc", 0.0),
                    )
                    model_hold = get_hold_estimate(self._db, self._current_temp)
                    if model_hold is not None and model_hold > 0:
                        rrc._integral = model_hold
                        logger.info(
                            f"Ramp rate ctrl: integral seeded at {model_hold:.1f}% "
                            f"from thermal model at {self._current_temp:.0f}°C"
                        )
                    self._ramp_rate_ctrl   = rrc
                    self._ramp_rate_target = rate_c
                    logger.info(
                        f"Segment {from_idx} → {to_idx}: RAMP "
                        f"rate={rate_c:.2f}°C/min target={seg.target_temp:.0f}°C "
                        f"— rate controller armed "
                        f"(kp={gains['kp']:.4f}, ki={gains['ki']:.6f})"
                    )
                else:
                    # Soak — switch back to PID.
                    # Seed the PID integral with the best available estimate of
                    # hold power at the soak temperature, in priority order:
                    #   1. Thermal model (continuously refined from real runs)
                    #   2. Ramp tune zone hold_pct (measured during autotune stabilize)
                    #   3. Conservative fallback
                    soak_temp = self._current_temp
                    hold_output = get_hold_estimate(self._db, soak_temp)
                    source = "thermal model"
                    if hold_output is None or hold_output <= 0:
                        gains = get_ramp_tune_params(self._db, soak_temp)
                        hold_output = gains.get("hold_pct", 0.0)
                        source = "ramp tune zone"
                    if hold_output <= 0:
                        # No calibration data at all — use 15% as a safe start
                        hold_output = 15.0
                        source = "fallback default"
                    self._ramp_rate_ctrl       = None
                    self._approach_pid_engaged = True
                    self._approach_phase       = "pid"
                    self._pid.reset()
                    self._pid._integral         = hold_output
                    self._pid._last_time        = None
                    self._pid._last_measurement = self._current_temp
                    logger.info(
                        f"Segment {from_idx} → {to_idx}: SOAK — PID seeded at "
                        f"{hold_output:.1f}% ({source})"
                    )
            else:
                self._ramp_rate_ctrl = None
