"""
program_engine.py — Ramp & soak program execution engine.

A "program" is a list of segments:
  - RAMP: move temperature from current to target at a given rate (°C/min)
  - SOAK: hold temperature for a given duration (minutes)

Programs are stored as JSON in SQLite via program_store.py.
"""

import time
import json
import logging
from dataclasses import dataclass, field, asdict
from typing import List, Optional, Callable
from enum import Enum

logger = logging.getLogger("program_engine")


class SegmentType(str, Enum):
    RAMP = "ramp"
    SOAK = "soak"


@dataclass
class Segment:
    type: SegmentType
    target_temp: float      # °C — target temp at end of ramp, or hold temp for soak
    rate_per_min: float = 0.0   # °C/min (used for RAMP)
    duration_min: float = 0.0   # minutes (used for SOAK)

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "Segment":
        return cls(
            type=SegmentType(d["type"]),
            target_temp=d["target_temp"],
            rate_per_min=d.get("rate_per_min", 0.0),
            duration_min=d.get("duration_min", 0.0),
        )


@dataclass
class Program:
    id: Optional[int]
    name: str
    description: str
    segments: List[Segment]
    created_at: float = field(default_factory=time.time)
    updated_at: float = field(default_factory=time.time)

    def to_dict(self):
        return {
            "id": self.id,
            "name": self.name,
            "description": self.description,
            "segments": [s.to_dict() for s in self.segments],
            "created_at": self.created_at,
            "updated_at": self.updated_at,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "Program":
        return cls(
            id=d.get("id"),
            name=d["name"],
            description=d.get("description", ""),
            segments=[Segment.from_dict(s) for s in d.get("segments", [])],
            created_at=d.get("created_at", time.time()),
            updated_at=d.get("updated_at", time.time()),
        )

    def estimated_duration_min(self, start_temp: float = 20.0) -> float:
        """Estimate total program duration in minutes.
        Continuous soaks (duration_min=-1) contribute 0 to the estimate."""
        total = 0.0
        current = start_temp
        for seg in self.segments:
            if seg.type == SegmentType.RAMP:
                delta = abs(seg.target_temp - current)
                rate = seg.rate_per_min if seg.rate_per_min > 0 else 10.0
                total += delta / rate
                current = seg.target_temp
            else:  # SOAK
                if seg.duration_min >= 0:
                    total += seg.duration_min
                # -1 = continuous; not counted in estimate
        return total


class ProgramRunState(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    PAUSED = "paused"
    COMPLETED = "completed"
    ABORTED = "aborted"
    AUTOTUNE = "autotune"


@dataclass
class RunStatus:
    state: ProgramRunState
    program_name: str
    segment_index: int
    segment_total: int
    segment_type: str
    segment_progress_pct: float
    overall_progress_pct: float
    setpoint: float
    current_temp: float
    elapsed_min: float
    remaining_min: float
    pid_output: float


class ProgramRunner:
    """
    Executes a ramp/soak program, computing the live setpoint at any moment.
    Call `tick(current_temp, now)` periodically → returns current setpoint.
    """

    def __init__(self):
        self._program: Optional[Program] = None
        self._state = ProgramRunState.IDLE
        self._seg_idx = 0
        self._seg_start_time: float = 0.0
        self._seg_start_temp: float = 0.0
        self._run_start_time: float = 0.0
        self._pause_start: Optional[float] = None
        self._paused_duration: float = 0.0
        self._setpoint: float = 0.0
        self._on_complete: Optional[Callable] = None
        self._on_segment_change: Optional[Callable] = None

    def load(self, program: Program, start_temp: float, on_complete=None, on_segment_change=None):
        self._program = program
        self._state = ProgramRunState.IDLE
        self._seg_idx = 0
        self._seg_start_temp = start_temp
        self._setpoint = start_temp
        self._on_complete = on_complete
        self._on_segment_change = on_segment_change
        logger.info(f"Loaded program: {program.name} ({len(program.segments)} segments)")

    def start(self, current_temp: float):
        if self._program is None:
            raise RuntimeError("No program loaded")
        now = time.time()
        self._state = ProgramRunState.RUNNING
        self._seg_idx = 0
        self._seg_start_time = now
        self._seg_start_temp = current_temp
        self._run_start_time = now
        self._paused_duration = 0.0
        self._setpoint = current_temp
        logger.info(f"Program started: {self._program.name}")

    def pause(self):
        if self._state == ProgramRunState.RUNNING:
            self._state = ProgramRunState.PAUSED
            self._pause_start = time.time()
            logger.info("Program paused")

    def resume(self):
        if self._state == ProgramRunState.PAUSED and self._pause_start:
            paused = time.time() - self._pause_start
            self._paused_duration += paused
            self._seg_start_time += paused
            self._pause_start = None
            self._state = ProgramRunState.RUNNING
            logger.info("Program resumed")

    def abort(self):
        self._state = ProgramRunState.ABORTED
        logger.info("Program aborted")

    def tick(self, current_temp: float, now: float = None) -> float:
        """Call this every control loop iteration. Returns current setpoint."""
        if now is None:
            now = time.time()

        if self._state != ProgramRunState.RUNNING:
            return self._setpoint

        program = self._program
        if self._seg_idx >= len(program.segments):
            self._state = ProgramRunState.COMPLETED
            if self._on_complete:
                self._on_complete()
            logger.info("Program completed")
            return self._setpoint

        seg = program.segments[self._seg_idx]
        elapsed_in_seg = now - self._seg_start_time

        if seg.type == SegmentType.RAMP:
            # Compute time needed to reach target from seg_start_temp
            delta_temp = seg.target_temp - self._seg_start_temp
            if seg.rate_per_min <= 0:
                # Instant jump
                self._setpoint = seg.target_temp
                self._advance_segment(seg.target_temp, now)
            else:
                duration_s = abs(delta_temp) / seg.rate_per_min * 60.0
                if duration_s <= 0 or elapsed_in_seg >= duration_s:
                    self._setpoint = seg.target_temp
                    self._advance_segment(seg.target_temp, now)
                else:
                    progress = elapsed_in_seg / duration_s
                    self._setpoint = self._seg_start_temp + delta_temp * progress

        elif seg.type == SegmentType.SOAK:
            self._setpoint = seg.target_temp
            if seg.duration_min == -1:
                pass  # Continuous soak — hold forever until aborted
            else:
                duration_s = seg.duration_min * 60.0
                if elapsed_in_seg >= duration_s:
                    self._advance_segment(seg.target_temp, now)

        return self._setpoint

    def _advance_segment(self, end_temp: float, now: float):
        prev_idx = self._seg_idx
        self._seg_idx += 1
        self._seg_start_time = now
        self._seg_start_temp = end_temp
        if self._on_segment_change and self._seg_idx < len(self._program.segments):
            self._on_segment_change(prev_idx, self._seg_idx)

    def get_status(self, current_temp: float, pid_output: float) -> RunStatus:
        now = time.time()
        program = self._program

        if program is None or self._state == ProgramRunState.IDLE:
            return RunStatus(
                state=self._state, program_name="", segment_index=0,
                segment_total=0, segment_type="", segment_progress_pct=0,
                overall_progress_pct=0, setpoint=self._setpoint,
                current_temp=current_temp, elapsed_min=0, remaining_min=0,
                pid_output=pid_output,
            )

        total_segs = len(program.segments)
        seg_type = ""
        seg_progress = 0.0

        if self._seg_idx < total_segs:
            seg = program.segments[self._seg_idx]
            seg_type = seg.type.value
            elapsed_seg = now - self._seg_start_time
            if seg.type == SegmentType.RAMP:
                delta = abs(seg.target_temp - self._seg_start_temp)
                rate = seg.rate_per_min if seg.rate_per_min > 0 else 10.0
                dur = delta / rate * 60.0
                seg_progress = min(100.0, elapsed_seg / dur * 100.0) if dur > 0 else 100.0
            else:
                if seg.duration_min == -1:
                    seg_progress = 0.0  # Continuous — no meaningful percentage
                else:
                    dur = seg.duration_min * 60.0
                    seg_progress = min(100.0, elapsed_seg / dur * 100.0) if dur > 0 else 100.0

        # Overall progress
        overall = (self._seg_idx / total_segs * 100.0 + seg_progress / total_segs) if total_segs > 0 else 0.0

        elapsed_total = (now - self._run_start_time - self._paused_duration) / 60.0
        estimated_total = program.estimated_duration_min(self._seg_start_temp)
        remaining = max(0.0, estimated_total - elapsed_total)

        return RunStatus(
            state=self._state,
            program_name=program.name,
            segment_index=self._seg_idx,
            segment_total=total_segs,
            segment_type=seg_type,
            segment_progress_pct=round(seg_progress, 1),
            overall_progress_pct=round(overall, 1),
            setpoint=round(self._setpoint, 1),
            current_temp=round(current_temp, 1),
            elapsed_min=round(elapsed_total, 1),
            remaining_min=round(remaining, 1),
            pid_output=round(pid_output, 1),
        )

    @property
    def state(self):
        return self._state

    @property
    def setpoint(self):
        return self._setpoint


# Built-in programs for common knife steels
BUILTIN_PROGRAMS = [
    Program(
        id=None,
        name="1084 High Carbon",
        description="Heat treat for 1084 carbon steel. Normalize, then harden at 1475°F (802°C).",
        segments=[
            Segment(SegmentType.RAMP, 371, rate_per_min=8),    # Ramp to 700°F (stress relief)
            Segment(SegmentType.SOAK, 371, duration_min=15),   # Soak 15 min
            Segment(SegmentType.RAMP, 802, rate_per_min=10),   # Ramp to 1475°F
            Segment(SegmentType.SOAK, 802, duration_min=10),   # Soak 10 min then quench
        ]
    ),
    Program(
        id=None,
        name="D2 Tool Steel",
        description="D2 tool steel hardening. Austenitize at 1010°C (1850°F).",
        segments=[
            Segment(SegmentType.RAMP, 500, rate_per_min=5),
            Segment(SegmentType.SOAK, 500, duration_min=20),
            Segment(SegmentType.RAMP, 800, rate_per_min=5),
            Segment(SegmentType.SOAK, 800, duration_min=15),
            Segment(SegmentType.RAMP, 1010, rate_per_min=5),
            Segment(SegmentType.SOAK, 1010, duration_min=30),
        ]
    ),
    Program(
        id=None,
        name="Temper 375°F",
        description="Tempering cycle at 190°C (375°F) for most carbon steels.",
        segments=[
            Segment(SegmentType.RAMP, 190, rate_per_min=3),
            Segment(SegmentType.SOAK, 190, duration_min=60),
            Segment(SegmentType.RAMP, 190, rate_per_min=3),  # Second temper
            Segment(SegmentType.SOAK, 190, duration_min=60),
        ]
    ),
    Program(
        id=None,
        name="Normalize 1084",
        description="Normalization cycle for 1084 to relieve stress before final heat treat.",
        segments=[
            Segment(SegmentType.RAMP, 870, rate_per_min=10),
            Segment(SegmentType.SOAK, 870, duration_min=10),
        ]
    ),
]
