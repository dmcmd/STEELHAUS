"""
esp32_bridge.py — Serial communication bridge between RPi and ESP32.

Sends PID output commands to ESP32, receives temperature readings.
Runs in its own thread.
"""

import serial
import serial.tools.list_ports
import os
import glob
import json
import time
import threading
import logging
from dataclasses import dataclass, field
from typing import Optional, Callable

logger = logging.getLogger("esp32_bridge")


@dataclass
class ESP32State:
    temp: float = 0.0
    cold_junction: float = 0.0
    ssr_duty: int = 0          # Both SSRs always share this single value
    contactor: bool = False    # True = contactor energized, AC power live at SSRs
    mosfet_duty: int = 0
    fault_code: int = 0
    estop: bool = False
    watchdog_tripped: bool = False
    door_open: bool = False
    last_update: float = 0.0
    connected: bool = False


class ESP32Bridge:
    """
    Thread-safe bridge to ESP32 over serial/USB.

    Usage:
        bridge = ESP32Bridge("/dev/ttyAMA0", 115200)
        bridge.start()
        bridge.set_output(duty=50, mosfet=0)
        state = bridge.state
    """

    def __init__(
        self,
        port: str = "/dev/ttyAMA0",
        baud: int = 115200,
        on_temp_update: Optional[Callable] = None,
        on_fault: Optional[Callable] = None,
    ):
        self.port = port
        self.baud = baud
        self._on_temp_update = on_temp_update
        self._on_fault = on_fault

        self._serial: Optional[serial.Serial] = None
        self._state = ESP32State()
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._send_queue: list = []
        self._priority_cmd: Optional[str] = None  # Contactor/estop commands — not overwritable
        self._queue_lock = threading.Lock()

        # Ping thread
        self._ping_thread: Optional[threading.Thread] = None
        self._last_send = 0.0

        # 3-sample median filter to suppress impulse noise from SSR switching.
        # Single-sample spikes (in either direction) cannot affect the median —
        # they sit at one end of the 3-value sorted window and are ignored.
        # During a ramp all 3 samples increase monotonically so the median
        # tracks accurately with at most 1-sample lag (~1 second).
        self._temp_window: list = []   # last 3 raw readings

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True, name="esp32-rx")
        self._thread.start()
        self._ping_thread = threading.Thread(target=self._ping_loop, daemon=True, name="esp32-ping")
        self._ping_thread.start()
        logger.info(f"ESP32 bridge started on {self.port}")

    def stop(self):
        self._running = False
        self.emergency_stop()
        if self._serial and self._serial.is_open:
            self._serial.close()

    def set_output(self, duty: int = 0, mosfet: int = 0):
        """Set heater output duty cycle (0-100). Applied identically to both SSRs."""
        cmd = json.dumps({"cmd": "set", "duty": duty, "mosfet": mosfet}) + "\n"
        self._enqueue(cmd)

    def set_contactor(self, on: bool):
        """Energize or de-energize the contactor relay.
        
        The contactor is the master AC disconnect upstream of both SSRs.
        Turning it on allows AC power to reach the heating element.
        Turning it off physically disconnects mains regardless of SSR state.
        The ESP32 will refuse to energize the contactor if any fault is active.
        """
        cmd = json.dumps({"cmd": "contactor", "on": on}) + "\n"
        self._enqueue(cmd)

    def emergency_stop(self):
        cmd = json.dumps({"cmd": "estop"}) + "\n"
        self._enqueue(cmd)

    def reset(self):
        cmd = json.dumps({"cmd": "reset"}) + "\n"
        self._enqueue(cmd)

    def ping(self):
        cmd = json.dumps({"cmd": "ping"}) + "\n"
        self._enqueue(cmd)

    def _enqueue(self, msg: str):
        with self._queue_lock:
            data = json.loads(msg.strip())
            cmd  = data.get("cmd")
            if cmd in ("contactor", "estop", "reset"):
                # Safety-critical commands always go to priority slot.
                # These must never be overwritten by anything.
                self._priority_cmd = msg
            elif cmd == "ping":
                # Ping only goes to priority slot if there is no
                # safety-critical command already waiting there.
                # This prevents a keepalive ping from silently erasing
                # a contactor-on or estop command that hasn't been sent yet.
                existing = self._priority_cmd
                if existing is None:
                    self._priority_cmd = msg
                else:
                    try:
                        existing_cmd = json.loads(existing.strip()).get("cmd")
                        if existing_cmd not in ("contactor", "estop", "reset"):
                            self._priority_cmd = msg
                        # else: safety command is pending — let it go first, drop the ping
                    except Exception:
                        self._priority_cmd = msg
            else:
                # Regular commands (set, etc.) — only keep the latest
                self._send_queue = [msg]

    def _next_cmd(self):
        """Return the next command to send, priority commands first."""
        with self._queue_lock:
            if self._priority_cmd:
                cmd = self._priority_cmd
                self._priority_cmd = None
                return cmd
            if self._send_queue:
                return self._send_queue.pop(0)
            return None

    def _find_port(self, timeout=5):
        """Wait up to timeout seconds for the serial port to appear."""
        deadline = time.time() + timeout
        while time.time() < deadline and self._running:
            if os.path.exists(self.port):
                return self.port
            logger.info(f"Waiting for {self.port}...")
            time.sleep(1.0)
        logger.warning(f"{self.port} not found after waiting — will retry")
        return None

    def _run_loop(self):
        while self._running:
            active_port = None
            try:
                active_port = self._find_port(timeout=60)
                if not active_port:
                    time.sleep(3.0)
                    continue

                # Open port with DTR/RTS disabled from the start to prevent ESP32 reset
                self._serial = serial.Serial()
                self._serial.port = active_port
                self._serial.baudrate = self.baud
                self._serial.timeout = 1.0
                self._serial.dsrdtr = False
                self._serial.rtscts = False
                self._serial.dtr = False
                self._serial.rts = False
                self._serial.open()
                self._serial.reset_input_buffer()
                with self._lock:
                    self._state.connected = True
                logger.info(f"Connected to ESP32 on {active_port}")

                consecutive_errors = 0
                while self._running:
                    # Send queued commands — priority slot first
                    msg = self._next_cmd()
                    if msg:
                        try:
                            self._serial.write(msg.encode("utf-8"))
                            self._last_send = time.time()
                        except Exception as e:
                            logger.error(f"Serial write error: {e}")
                            break

                    # Read response
                    try:
                        line = self._serial.readline().decode("utf-8", errors="ignore").strip()
                        if line:
                            self._parse_report(line)
                            consecutive_errors = 0
                        else:
                            # Timeout — no data received, check if ESP32 is alive
                            consecutive_errors += 1
                            if consecutive_errors == 10:
                                logger.warning(f"No data from ESP32 for 10s — still waiting...")
                            if consecutive_errors > 30:
                                logger.warning("No data from ESP32 for 30s — reconnecting")
                                break
                    except Exception as e:
                        logger.error(f"Serial read error: {e}")
                        break

            except serial.SerialException as e:
                logger.error(f"Serial connection error: {e}")
            finally:
                # Always clean up before reconnecting
                with self._lock:
                    self._state.connected = False
                try:
                    if self._serial and self._serial.is_open:
                        self._serial.close()
                except Exception:
                    pass
                self._serial = None
                time.sleep(3.0)

    def _ping_loop(self):
        """Send keepalive ping to reset ESP32 watchdog when no set commands are queued."""
        while self._running:
            time.sleep(2.0)
            if time.time() - self._last_send > 3.0:
                cmd = json.dumps({"cmd": "ping"}) + "\n"
                self._enqueue(cmd)

    def _parse_report(self, line: str):
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            return

        # Ignore 0.0°C readings with no fault — MAX31856 not ready yet
        temp_raw = data.get("temp", 0.0)
        fault = data.get("fault", 0)
        if temp_raw == 0.0 and fault == 0:
            return

        # 3-sample median filter — eliminates single-sample spikes completely.
        # Needs only 3 readings; outputs the middle value after sorting.
        self._temp_window.append(temp_raw)
        if len(self._temp_window) > 3:
            self._temp_window.pop(0)
        temp_filtered = sorted(self._temp_window)[len(self._temp_window) // 2]

        now = time.time()
        with self._lock:
            prev_fault = self._state.fault_code
            self._state.temp = temp_filtered     # smoothed value used by controller
            self._state.cold_junction = data.get("cj", 0.0)
            self._state.ssr_duty = data.get("ssr_duty", 0)
            self._state.contactor = data.get("contactor", False)
            self._state.mosfet_duty = data.get("mosfet", 0)
            self._state.fault_code = data.get("fault", 0)
            self._state.estop = data.get("estop", False)
            self._state.watchdog_tripped = data.get("wdog", False)
            self._state.door_open = data.get("door", False)
            self._state.last_update = now
            new_fault = self._state.fault_code

        if self._on_temp_update:
            self._on_temp_update(self._state.temp, now)

        if new_fault != 0 and new_fault != prev_fault and self._on_fault:
            self._on_fault(new_fault)

    @property
    def state(self) -> ESP32State:
        with self._lock:
            # Return a copy
            s = self._state
            return ESP32State(
                temp=s.temp, cold_junction=s.cold_junction,
                ssr_duty=s.ssr_duty, contactor=s.contactor,
                mosfet_duty=s.mosfet_duty, fault_code=s.fault_code,
                estop=s.estop, watchdog_tripped=s.watchdog_tripped,
                door_open=s.door_open,
                last_update=s.last_update, connected=s.connected,
            )

    @property
    def temperature(self) -> float:
        with self._lock:
            return self._state.temp

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._state.connected


FAULT_MESSAGES = {
    0x01: "Open thermocouple",
    0x02: "Overvoltage / undervoltage input",
    0x04: "Thermocouple temperature too low",
    0x08: "Thermocouple temperature too high",
    0x10: "Cold junction temperature too low",
    0x20: "Cold junction temperature too high",
    0x40: "Thermocouple out of range",
    0x80: "Cold junction out of range",
}


def decode_fault(fault_code: int) -> list:
    messages = []
    for bit, msg in FAULT_MESSAGES.items():
        if fault_code & bit:
            messages.append(msg)
    return messages
