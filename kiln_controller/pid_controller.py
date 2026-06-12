"""
pid_controller.py — PID controller and relay-feedback autotune.

PID implementation uses:
  - Derivative on measurement (not error) to avoid derivative kick on setpoint changes
  - Low-pass filter on derivative term to suppress thermocouple noise (alpha=0.3 default)
  - Anti-windup via integral clamping
  - Bumpless parameter updates

Autotune uses the Åström–Hägglund relay feedback method:
  - Drives the output between relay_output_high and relay_output_low
  - Detects oscillation peaks and valleys in the process variable
  - After required_cycles complete oscillations, calculates ultimate gain (Ku)
    and ultimate period (Tu)
  - Applies one of four tuning rules to produce Kp, Ki, Kd
"""

import time
import math
import logging
from dataclasses import dataclass
from typing import Optional

logger = logging.getLogger("pid_controller")


@dataclass
class PIDParams:
    kp: float
    ki: float
    kd: float
    output_min: float = 0.0
    output_max: float = 100.0
    sample_time: float = 1.0
    derivative_filter_alpha: float = 0.3   # Low-pass filter on derivative term.
                                            # 1.0 = no filtering, 0.1 = heavy filtering.
                                            # 0.3 is a good default for thermocouples.


@dataclass
class AutotuneResult:
    kp: float
    ki: float
    kd: float
    ku: float          # Ultimate gain
    tu: float          # Ultimate period (seconds)
    method: str


class PIDController:
    """
    PID controller with derivative-on-measurement and anti-windup.
    """

    def __init__(self, params: PIDParams):
        self._params = params
        self._integral = 0.0
        self._last_measurement = None
        self._last_time = None
        self._derivative_filter = 0.0   # Low-pass filtered derivative value

    @property
    def params(self) -> PIDParams:
        return self._params

    def set_params(self, params: PIDParams):
        # Bumpless transfer: rescale integral so output doesn't jump
        if self._params.ki > 0 and params.ki > 0:
            self._integral *= self._params.ki / params.ki
        self._params = params

    def reset(self):
        self._integral = 0.0
        self._last_measurement = None
        self._last_time = None
        self._derivative_filter = 0.0

    def update(self, setpoint: float, measurement: float, now: float = None) -> float:
        if now is None:
            now = time.time()

        p = self._params

        if self._last_time is None:
            self._last_time = now
            self._last_measurement = measurement
            return 0.0

        dt = now - self._last_time
        if dt <= 0:
            return 0.0

        error = setpoint - measurement

        # Proportional term
        p_term = p.kp * error

        # Integral term with anti-windup clamping
        self._integral += p.ki * error * dt
        self._integral = max(p.output_min, min(p.output_max, self._integral))

        # Derivative on measurement (avoids kick when setpoint changes).
        # Low-pass filtered to suppress thermocouple noise amplification.
        # alpha=1.0 means no filtering; alpha=0.3 gives smooth behaviour
        # while still reacting to real temperature changes.
        alpha = p.derivative_filter_alpha
        raw_d_measurement = (measurement - self._last_measurement) / dt
        self._derivative_filter = (alpha * raw_d_measurement
                                   + (1.0 - alpha) * self._derivative_filter)
        d_term = -p.kd * self._derivative_filter

        output = p_term + self._integral + d_term
        output = max(p.output_min, min(p.output_max, output))

        self._last_measurement = measurement
        self._last_time = now

        return output


class RelayAutotuner:
    """
    Relay-feedback autotuner using the Åström–Hägglund method.

    The relay switches output between relay_output_high and relay_output_low,
    inducing sustained oscillations in the process variable.
    After observing required_cycles complete oscillations, it calculates
    the ultimate gain (Ku) and ultimate period (Tu) and derives PID parameters.
    """

    def __init__(self,
                 setpoint: float,
                 relay_output_high: float = 70.0,
                 relay_output_low: float = 5.0,
                 noise_band: float = 0.5,
                 required_cycles: int = 5):
        self.setpoint = setpoint
        self.relay_high = relay_output_high
        self.relay_low = relay_output_low
        self.noise_band = noise_band
        self.required_cycles = required_cycles

        self._relay_state = True    # Start in high state — heat first, then oscillate
        self._peak_high: list = []  # Recorded high peaks (temp above setpoint)
        self._peak_low:  list = []  # Recorded low peaks  (temp below setpoint)
        self._last_above = False
        self._cycle_count = 0
        self._peak_times: list = []
        self._last_peak_time: Optional[float] = None
        self._last_measurement = None
        self._done = False
        self._result: Optional[AutotuneResult] = None

    @property
    def done(self) -> bool:
        return self._done

    @property
    def cycle_count(self) -> int:
        return self._cycle_count

    @property
    def peak_high(self) -> Optional[float]:
        return max(self._peak_high) if self._peak_high else None

    @property
    def peak_low(self) -> Optional[float]:
        return min(self._peak_low) if self._peak_low else None

    def update(self, measurement: float, now: float = None) -> float:
        if now is None:
            now = time.time()

        if self._done:
            return self.relay_low

        above_setpoint = measurement > (self.setpoint + self.noise_band)
        below_setpoint = measurement < (self.setpoint - self.noise_band)

        # Relay switching logic
        if above_setpoint and self._relay_state:
            # Was heating, now above setpoint — switch to low
            self._relay_state = False
            self._record_peak(measurement, now, is_high=True)

        elif below_setpoint and not self._relay_state:
            # Was cooling, now below setpoint — switch to high
            self._relay_state = True
            self._record_peak(measurement, now, is_high=False)

        self._last_measurement = measurement

        return self.relay_high if self._relay_state else self.relay_low

    def _record_peak(self, value: float, now: float, is_high: bool):
        if is_high:
            self._peak_high.append(value)
        else:
            self._peak_low.append(value)
            self._cycle_count += 1
            ph = f"{max(self._peak_high):.1f}" if self._peak_high else "?"
            pl = f"{min(self._peak_low):.1f}" if self._peak_low else "?"
            logger.info(f"Autotune cycle {self._cycle_count}/{self.required_cycles} "
                        f"— peak high: {ph}, peak low: {pl}")

        if self._last_peak_time is not None:
            period = now - self._last_peak_time
            self._peak_times.append(period * 2)  # Half-period × 2 = full period
        self._last_peak_time = now

        if self._cycle_count >= self.required_cycles:
            self._done = True
            logger.info("Autotune: sufficient cycles collected, calculating parameters")

    def get_result(self, method: str = "some-overshoot") -> AutotuneResult:
        """
        Calculate PID parameters from collected oscillation data.

        Ku = 4d / (π × A)
          where d = relay amplitude (high - low) / 2
                A = process oscillation amplitude (peak_high - peak_low) / 2

        Tu = mean of observed oscillation periods
        """
        if not self._done:
            raise RuntimeError("Autotune not complete")

        # Relay amplitude
        d = (self.relay_high - self.relay_low) / 2.0

        # Process oscillation amplitude
        if not self._peak_high or not self._peak_low:
            raise RuntimeError("Insufficient peak data")

        peak_high_mean = sum(self._peak_high) / len(self._peak_high)
        peak_low_mean  = sum(self._peak_low)  / len(self._peak_low)
        A = (peak_high_mean - peak_low_mean) / 2.0

        if A <= 0:
            raise RuntimeError("Invalid oscillation amplitude — check thermocouple and wiring")

        # Ultimate gain and period
        Ku = (4.0 * d) / (math.pi * A)

        if len(self._peak_times) > 1:
            Tu = sum(self._peak_times) / len(self._peak_times)
        elif self._peak_times:
            Tu = self._peak_times[0]
        else:
            raise RuntimeError("No period data collected")

        logger.info(f"Autotune result: Ku={Ku:.4f}, Tu={Tu:.1f}s, method={method}")

        kp, ki, kd = self._apply_tuning_rule(Ku, Tu, method)

        return AutotuneResult(kp=round(kp, 6), ki=round(ki, 6), kd=round(kd, 6),
                              ku=round(Ku, 4), tu=round(Tu, 2), method=method)

    @staticmethod
    def _apply_tuning_rule(Ku: float, Tu: float, method: str):
        """
        Apply a tuning rule to Ku and Tu to produce Kp, Ki, Kd.

        steelhaus is the recommended default for this oven — derived empirically
        from real heat-treat runs. It uses a conservative Kp (Ku/6.7) with a
        small derivative term (Kp*Tu/9) filtered by the PID's derivative low-pass
        filter. Produces a clean single overshoot of ~5°F then straight to setpoint.

        Kd is set to 0 for tyreus-luyben, some-overshoot, and no-overshoot since
        they were originally tuned without it. Only steelhaus and ziegler-nichols
        include a Kd term.
        """
        if method == "steelhaus":
            # Empirically tuned for this oven's thermal mass and element characteristics.
            # Kp = Ku/6.7 — conservative to avoid overshoot
            # Ki = 2*Kp/Tu — standard integral ratio
            # Kd = Kp*Tu/9 — small derivative, capped at 15*Kp to prevent
            #      excessive derivative action at low temperatures where Tu is very long
            kp = Ku   / 6.7
            ki = 2.0  * kp / Tu
            kd = min(kp * Tu / 9.0, kp * 15.0)

        elif method == "ziegler-nichols":
            kp = 0.6  * Ku
            ki = 2.0  * kp / Tu
            kd = kp   * Tu / 8.0

        elif method == "tyreus-luyben":
            kp = Ku   / 3.2
            ki = kp   / (2.87 * Tu)
            kd = 0.0

        elif method == "some-overshoot":
            kp = Ku   / 3.0
            ki = 2.0  * kp / Tu
            kd = 0.0

        elif method == "no-overshoot":
            kp = Ku   / 5.0
            ki = 2.0  * kp / Tu
            kd = 0.0

        else:
            # Default to steelhaus
            kp = Ku   / 6.7
            ki = 2.0  * kp / Tu
            kd = kp   * Tu / 9.0

        return kp, ki, kd


class RampRateController:
    """
    Rate-based controller for program ramp segments.

    Instead of chasing a moving setpoint, this controller:
      1. Estimates the actual rate of temperature rise (°/min) using a
         linear regression over a rolling window of recent samples.
         Regression is much more noise-robust than tick-to-tick difference.
      2. Computes  rate_error = target_rate - actual_rate
      3. Drives output with a PI on that rate error.

    The integral term handles steady-state offset — an oven always needs
    some base output just to sustain a climb against radiation losses, and
    the integral accumulates that automatically.

    Tuning parameters (defaults work well for a brick kiln):
      kp_rate  — how aggressively to react to rate error.
                 Start at 2.0: each 1°/min of rate error adds 2% output.
      ki_rate  — integral wind-up speed.
                 Start at 0.15: slow enough not to overshoot on startup.
      window_s — seconds of history used for slope estimation.
                 60s gives a stable reading; reduce if ramp rate is very fast.
      output_min/max — clamp output (0–100%).
    """

    # Starting gains — sized for a typical kiln with brick/refractory mass.
    # kp=2.0: 2% output per 1°C/min rate error (e.g. 10°C/min error → 20% correction)
    # ki=0.10: integral winds up at 0.1%/s per °C/min of sustained error
    # These are overridden by tuned zone values from the ramp autotune.
    DEFAULT_KP         = 2.0
    DEFAULT_KI         = 0.10
    DEFAULT_POS_WEIGHT = 0.5   # °C/min correction per °C of position error

    def __init__(self,
                 target_rate_per_min: float,
                 kp: float = DEFAULT_KP,
                 ki: float = DEFAULT_KI,
                 window_s: float = 60.0,
                 output_min: float = 0.0,
                 output_max: float = 100.0,
                 K_proc: float = 0.0,
                 pos_weight: float = DEFAULT_POS_WEIGHT):
        self.target_rate = target_rate_per_min
        self.kp          = kp
        self.ki          = ki
        self.window_s    = window_s
        self.output_min  = output_min
        self.output_max  = output_max
        self.K_proc      = K_proc       # used to size startup output
        self.pos_weight  = pos_weight   # position correction weight

        self._samples: list = []
        self._integral: float = 0.0
        self._last_time: Optional[float] = None
        self._last_output: float = 0.0

    def reset(self):
        self._samples.clear()
        self._integral = 0.0
        self._last_time = None
        self._last_output = 0.0

    @property
    def actual_rate(self) -> float:
        """Current estimated rate of rise in °/min (same unit as target_rate)."""
        return self._estimate_rate()

    def update(self, temp: float, now: float = None, setpoint: float = None) -> float:
        """
        Call once per control loop tick. Returns output 0–100.
        temp and setpoint must be in the same unit as target_rate_per_min (°C).

        Control strategy — blended rate + position error:
          The controller tracks two things simultaneously:
            1. Rate error:     target_rate - actual_rate  (°C/min)
            2. Position error: setpoint - temp            (°C above/below ramp line)

          Position error is converted to a rate-equivalent correction:
            pos_correction = position_error × pos_weight   (°C/min equivalent)

          The two are summed into a single blended error fed to the PI:
            blended_error = rate_error + pos_correction

          Effect:
          - Oven on the line, climbing at correct rate → both terms near zero → output steady
          - Oven below the line → positive pos_correction → output increases
          - Oven above the line → negative pos_correction → output decreases smoothly
          - Oven far above line → large negative correction → output drops significantly
            but never cuts to zero unless the oven is very far above

          This avoids bang-bang oscillation because output is always a smooth
          continuous function of both errors, never a hard switch.

        pos_weight: how strongly position error pulls toward the ramp line.
          Units: (°C/min correction) per (°C of position error).
          Default 0.5 means: 1°C above line → -0.5°C/min correction to blended error.
          At kp=2.3: that's 2.3 × 0.5 = 1.15% output reduction per °C of overshoot.
          Increase if the oven consistently runs above the line; decrease if it hunts.
        """
        if now is None:
            now = time.time()

        # Add sample and prune window
        self._samples.append((now, temp))
        cutoff = now - self.window_s
        self._samples = [(t, v) for t, v in self._samples if t >= cutoff]

        # Position correction — converts position error to a rate-equivalent term
        pos_correction = 0.0
        if setpoint is not None:
            pos_error = setpoint - temp           # positive = below line, negative = above
            pos_correction = pos_error * self.pos_weight  # °C/min equivalent

        # Need at least 10 seconds of data before trusting the slope estimate
        if len(self._samples) < 10 or (self._samples[-1][0] - self._samples[0][0]) < 10.0:
            # Size startup output from K: target_rate / K gives steady-state output.
            # Apply position correction even during startup so we don't blast past
            # the ramp line while waiting for enough data.
            if self.K_proc > 0:
                startup_out = min(80.0, max(5.0, self.target_rate / self.K_proc))
            else:
                startup_out = 30.0
            # Blend position correction into startup: above line reduces startup output
            startup_out += pos_correction * self.kp
            startup_out = max(self.output_min, min(self.output_max, startup_out))
            self._integral  = startup_out
            self._last_time = now
            self._last_output = startup_out
            return startup_out

        actual_rate   = self._estimate_rate()
        rate_error    = self.target_rate - actual_rate
        blended_error = rate_error + pos_correction

        # PI on blended error
        dt = 1.0 if self._last_time is None else max(0.1, now - self._last_time)
        self._integral += self.ki * blended_error * dt
        self._integral  = max(self.output_min, min(self.output_max, self._integral))

        output = self.kp * blended_error + self._integral
        output = max(self.output_min, min(self.output_max, output))

        self._last_time   = now
        self._last_output = output
        logger.debug(
            f"RampRate: actual={actual_rate:.2f}°/min target={self.target_rate:.2f}°/min "
            f"rate_err={rate_error:.2f} pos_corr={pos_correction:.2f} "
            f"blended={blended_error:.2f} integral={self._integral:.1f} output={output:.1f}%"
        )
        return output
        logger.debug(
            f"RampRate: actual={actual_rate:.2f}°/min target={self.target_rate:.2f}°/min "
            f"err={rate_error:.2f} pos_penalty={pos_penalty:.1f} "
            f"integral={self._integral:.1f} output={output:.1f}%"
        )
        return output

    def _estimate_rate(self) -> float:
        """
        Estimate current rate of rise (°/min) using least-squares linear regression
        over the sample window.  Much more noise-robust than (last-first)/dt.
        Returns 0.0 if insufficient data.
        """
        n = len(self._samples)
        if n < 2:
            return 0.0

        # Normalise timestamps to reduce floating-point error
        t0 = self._samples[0][0]
        xs = [s[0] - t0 for s in self._samples]   # seconds since window start
        ys = [s[1]       for s in self._samples]   # temperatures

        # Least-squares slope: β = (n·Σxy - Σx·Σy) / (n·Σx² - (Σx)²)
        sx  = sum(xs)
        sy  = sum(ys)
        sxy = sum(x * y for x, y in zip(xs, ys))
        sx2 = sum(x * x for x in xs)
        denom = n * sx2 - sx * sx
        if abs(denom) < 1e-9:
            return 0.0

        slope_per_sec = (n * sxy - sx * sy) / denom   # °/second
        return slope_per_sec * 60.0                    # convert to °/minute


class RampStepAutotuner:
    """
    Step-response ramp autotune for the rate controller.

    Procedure:
      Phase 1 — STABILIZE (passive):
        The controller's normal loop (approach zones + PID) heats to
        stabilize_setpoint exactly as it would for any manual setpoint.
        The autotuner just monitors temperature via monitor_stabilize()
        and waits for stability.  ready_to_step() returns True when stable;
        the controller then calls begin_step() to kick off Phase 2.

      Phase 2 — STEP:
        The autotuner takes over output: locks it to step_output (%) for
        step_duration_s seconds and records the temperature trajectory via
        update_step().

      Phase 3 — DONE:
        From the recorded trajectory, identify:
          K  = steady-state rate of rise (°C/min per % output)
          L  = lag time (seconds before measurable rise begins)
        Apply IMC tuning for an integrating (ramp) process:
          kp = 0.9 / (K * L)
          ki = kp / (3.3 * L)
        (Standard Lambda/IMC rule for FOPDT-integrating processes, lambda=L)

    All temperatures in °C internally.
    """

    PHASES = ("idle", "stabilize", "step", "done", "aborted")

    def __init__(self,
                 stabilize_setpoint_c: float,
                 step_output: float = 50.0,
                 step_duration_s: float = 360.0,
                 stable_band_c: float = 2.0,
                 stable_seconds: float = 60.0):
        self.stabilize_setpoint = stabilize_setpoint_c
        self.step_output        = step_output
        self.step_duration_s    = step_duration_s
        self.stable_band_c      = stable_band_c
        self.stable_seconds     = stable_seconds

        self.phase: str = "idle"
        self._stable_since: Optional[float] = None
        self._step_start:   Optional[float] = None
        self._step_samples: list = []   # list of (timestamp, temp_c)
        self._hold_outputs: list = []   # PID outputs during stable phase → hold_pct
        self.result: Optional[dict] = None

    @property
    def hold_pct(self) -> float:
        """Mean PID output during the stable phase — the hold power for this temperature."""
        if not self._hold_outputs:
            return 0.0
        return round(sum(self._hold_outputs) / len(self._hold_outputs), 2)

    def start(self):
        """
        Arm the autotuner.
        The controller keeps running its normal approach+PID loop to reach
        stabilize_setpoint — no change to heating behaviour during stabilize.
        """
        self.phase         = "stabilize"
        self._stable_since = None
        self._step_start   = None
        self._step_samples.clear()
        self._hold_outputs.clear()
        self.result        = None
        logger.info(
            f"RampStepAutotuner started — stabilizing at "
            f"{self.stabilize_setpoint:.1f}°C, step={self.step_output}%, "
            f"duration={self.step_duration_s}s"
        )

    def monitor_stabilize(self, temp_c: float, now: float = None,
                          pid_output: float = 0.0):
        """
        Call every tick while in stabilize phase.
        Does NOT return an output — the controller's normal approach+PID
        loop continues to manage heating.  Tracks stability window and
        records PID output once stable so hold_pct can be computed.
        pid_output: the current PID output % — recorded during stable window.
        """
        if now is None:
            now = time.time()
        if self.phase != "stabilize":
            return
        if abs(temp_c - self.stabilize_setpoint) <= self.stable_band_c:
            if self._stable_since is None:
                self._stable_since = now
                self._hold_outputs.clear()   # fresh measurement window
            # Record output during the stable window (last 30s before step)
            self._hold_outputs.append(pid_output)
            # Keep only the last 30 samples to avoid old approach-phase values
            if len(self._hold_outputs) > 30:
                self._hold_outputs.pop(0)
        else:
            self._stable_since = None
            self._hold_outputs.clear()

    def ready_to_step(self, now: float = None) -> bool:
        """True once temperature has been stable long enough to begin step."""
        if now is None:
            now = time.time()
        return (
            self.phase == "stabilize"
            and self._stable_since is not None
            and (now - self._stable_since) >= self.stable_seconds
        )

    def begin_step(self, now: float = None):
        """Transition stabilize → step.  Call when ready_to_step() is True."""
        if now is None:
            now = time.time()
        self.phase       = "step"
        self._step_start = now
        self._step_samples.clear()
        logger.info(
            f"RampStepAutotuner: stabilized — applying {self.step_output}% step"
        )

    def update_step(self, temp_c: float, now: float = None) -> float:
        """
        Call every tick during step phase.
        Returns the fixed step output to apply.
        Records samples and triggers calculation when duration is reached.
        """
        if now is None:
            now = time.time()
        if self.phase != "step":
            return 0.0
        self._step_samples.append((now, temp_c))
        if now - self._step_start >= self.step_duration_s:
            self._calculate_result()
            self.phase = "done"
            logger.info(f"RampStepAutotuner: step complete — {self.result}")
        return self.step_output

    @property
    def done(self) -> bool:
        return self.phase == "done"

    @property
    def elapsed_step_s(self) -> float:
        if self.phase == "step" and self._step_start:
            return time.time() - self._step_start
        return 0.0

    def abort(self):
        self.phase = "aborted"
        logger.info("RampStepAutotuner aborted")

    def _calculate_result(self):
        """
        Identify K and L from step-response samples, then compute rate controller gains.

        K (process gain, °C/min per % output):
          Linear regression over the last 60% of the step window where the
          temperature is rising steadily.  Divide slope by step_output to
          normalise to per-% output.

        L (lag time, seconds):
          How long after the step before the oven starts responding.
          We compute the instantaneous rate at each second using a SHORT
          (10-sample) backward difference, then find the first second where
          the rate exceeds 15% of the peak rate.  Using a short window avoids
          the artifact where a 30-sample window always reports t≈30s.

        Gains — direct synthesis for a rate (integrating) process:
          The IMC formula kp=0.9/(K*L) is correct in theory but produces
          very conservative gains for a kiln where L is short.  We use a
          more aggressive lambda=0.5*L (tighter closed-loop time constant)
          and a PI ratio suited to the oven's slow thermal response:

            kp = 1.0 / (K * max(L, 5.0))
            ki = kp / (4.0 * max(L, 5.0))

          At K=1.46, L=8s: kp=1/(1.46*8)=0.086 → too small.
          The fundamental issue is the rate-error unit: 1°C/min error at
          kp=0.086 gives 0.086% output.  For a kiln we need much higher Kp.

          Better approach: size Kp so that a 1°C/min rate error produces
          a meaningful corrective output.  From K: to accelerate by 1°C/min
          you need 1/K percent extra output.  A proportional gain of 2/K
          gives a 2% correction per 1°C/min error — responsive without
          oscillating.

            kp = 2.0 / K          (2% output per 1°C/min rate error)
            ki = kp / (5.0 * L)   (integral wind-up time = 5 lag-lengths)
        """
        samples = self._step_samples
        n = len(samples)
        if n < 20:
            logger.warning("RampStepAutotuner: insufficient samples for identification")
            self.result = {"kp": 2.0, "ki": 0.15, "K": 0.0, "L": 0.0, "method": "fallback"}
            return

        times = [s[0] - samples[0][0] for s in samples]   # seconds from step start
        temps = [s[1] for s in samples]

        # --- K: regression over the last 60% of step window ---
        split = int(n * 0.4)
        xs = times[split:]
        ys = temps[split:]
        ns = len(xs)
        sx  = sum(xs);  sy  = sum(ys)
        sxy = sum(x*y for x, y in zip(xs, ys))
        sx2 = sum(x*x for x in xs)
        denom = ns * sx2 - sx * sx
        if abs(denom) < 1e-9:
            rate_c_per_s = 0.001
        else:
            rate_c_per_s = (ns * sxy - sx * sy) / denom
        rate_c_per_min = rate_c_per_s * 60.0
        K_proc = rate_c_per_min / self.step_output   # °C/min per % output

        # --- L: short backward-difference to avoid window-size artifact ---
        # Use a 10-sample window so lag detection starts at t=10s, not t=30s
        SHORT = 10
        rates_short = []
        for i in range(SHORT, n):
            dt = times[i] - times[i - SHORT]
            if dt <= 0:
                continue
            r = (temps[i] - temps[i - SHORT]) / dt * 60.0  # °C/min
            rates_short.append((times[i], r))

        if rates_short:
            peak_rate = max(r for _, r in rates_short)
            threshold = peak_rate * 0.15   # 15% of peak = start of real response
            L_s = rates_short[-1][0]       # fallback
            for t_r, r in rates_short:
                if r >= threshold:
                    L_s = t_r
                    break
        else:
            L_s = 10.0

        L_s = max(L_s, 3.0)   # floor at 3s — prevent divide-by-tiny

        # --- Gains: direct synthesis sized for kiln thermal mass ---
        # kp = 2/K: 2% output correction per 1°C/min rate error
        # ki = kp/(5*L): integral winds up over 5 lag-lengths
        if K_proc > 0:
            kp = 2.0 / K_proc
            ki = kp  / (5.0 * L_s)
        else:
            kp = 2.0
            ki = 0.10

        # Sanity clamp — wide range, only catches truly degenerate cases
        kp = max(0.5, min(50.0, kp))
        ki = max(0.005, min(5.0, ki))

        self.result = {
            "kp":     round(kp, 4),
            "ki":     round(ki, 6),
            "K":      round(K_proc, 6),   # °C/min per % output
            "L":      round(L_s, 2),      # seconds
            "rate_c_per_min": round(rate_c_per_min, 3),
            "method": "imc-integrating",
        }
        logger.info(
            f"RampStepAutotuner result: K={K_proc:.4f} °C/min/%, L={L_s:.1f}s → "
            f"kp={kp:.4f}, ki={ki:.6f}"
        )
