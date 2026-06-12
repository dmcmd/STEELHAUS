"""
program_store.py — SQLite-backed storage for programs, settings, PID zones, and temperature logs.
"""

import sqlite3
import json
import time
import logging
from typing import List, Optional, Dict
from program_engine import Program, Segment, SegmentType, BUILTIN_PROGRAMS
from pid_controller import PIDParams

logger = logging.getLogger("program_store")

DB_PATH = "/home/pi/kiln_controller/data/kiln.db"


def get_db(path: str = DB_PATH) -> sqlite3.Connection:
    conn = sqlite3.connect(path, check_same_thread=False)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA journal_mode=WAL")
    return conn


def init_db(path: str = DB_PATH):
    conn = get_db(path)
    c = conn.cursor()

    c.execute("""
        CREATE TABLE IF NOT EXISTS programs (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            name TEXT NOT NULL,
            description TEXT DEFAULT '',
            segments_json TEXT NOT NULL,
            created_at REAL,
            updated_at REAL
        )
    """)

    c.execute("""
        CREATE TABLE IF NOT EXISTS settings (
            key TEXT PRIMARY KEY,
            value TEXT NOT NULL
        )
    """)

    c.execute("""
        CREATE TABLE IF NOT EXISTS temperature_log (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            ts REAL NOT NULL,
            temp REAL,
            setpoint REAL,
            pid_output REAL,
            program_name TEXT DEFAULT '',
            segment_index INTEGER DEFAULT -1,
            session_id INTEGER DEFAULT 1,
            door_open INTEGER DEFAULT 0
        )
    """)

    # sessions tracks discrete oven runs.
    # A new session is created on every startup (or manual "new session" action).
    # ended_at is NULL while the session is active, stamped on shutdown or new-session.
    c.execute("""
        CREATE TABLE IF NOT EXISTS sessions (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            label TEXT NOT NULL,
            started_at REAL NOT NULL,
            ended_at REAL
        )
    """)

    # Migrate: add session_id to pre-existing temperature_log tables
    try:
        c.execute("ALTER TABLE temperature_log ADD COLUMN session_id INTEGER DEFAULT 1")
        conn.commit()
    except Exception:
        pass  # Column already exists

    # Migrate: add door_open column
    try:
        c.execute("ALTER TABLE temperature_log ADD COLUMN door_open INTEGER DEFAULT 0")
        conn.commit()
    except Exception:
        pass  # Column already exists

    # Migrate: add protected column to sessions (0 = normal, 1 = protected from deletion)
    try:
        c.execute("ALTER TABLE sessions ADD COLUMN protected INTEGER DEFAULT 0")
        conn.commit()
    except Exception:
        pass  # Column already exists

    # Migrate: add hold_pct to ramp_tune_zones (steady-state output to hold temp)
    try:
        c.execute("ALTER TABLE ramp_tune_zones ADD COLUMN hold_pct REAL DEFAULT 0")
        conn.commit()
    except Exception:
        pass  # Column already exists

    # thermal_model stores the oven's continuously-refined hold-power curve.
    # Each row is an anchor point at a specific temperature.  Whenever the oven
    # is in verified steady-state PID, the measured output updates the nearest
    # anchor using an exponential moving average.  The model is used to seed the
    # PID integral when transitioning from a ramp to a soak, giving it an accurate
    # starting point rather than guessing or inheriting the ramp output.
    #
    # temp_c      — anchor temperature (°C), rounded to nearest MODEL_RESOLUTION_C
    # hold_pct    — current best estimate of hold power (%)
    # samples     — number of steady-state observations that went into this estimate
    # weight      — EMA weight; 1.0 on first sample, approaches 1.0 as samples grow
    # updated_at  — unix timestamp of last update
    c.execute("""
        CREATE TABLE IF NOT EXISTS thermal_model (
            id         INTEGER PRIMARY KEY AUTOINCREMENT,
            temp_c     REAL NOT NULL UNIQUE,
            hold_pct   REAL NOT NULL,
            samples    INTEGER NOT NULL DEFAULT 1,
            updated_at REAL NOT NULL
        )
    """)

    # Migrate: add resume_offset_c to door_recovery_zones.
    # New recovery logic: run at boost_pct until temp reaches setpoint - resume_offset_c,
    # then hand off to PID seeded from thermal model.
    # Default 5.6°C (~10°F) — resume PID when within 10°F of setpoint.
    try:
        c.execute("ALTER TABLE door_recovery_zones ADD COLUMN resume_offset_c REAL DEFAULT 5.6")
        conn.commit()
    except Exception:
        pass  # Column already exists
    # center_temp_c is the temperature the autotune was run at (in Celsius).
    # The controller interpolates between zones at runtime.
    c.execute("""
        CREATE TABLE IF NOT EXISTS pid_zones (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            center_temp_c REAL NOT NULL UNIQUE,
            kp REAL NOT NULL,
            ki REAL NOT NULL,
            kd REAL NOT NULL,
            ku REAL DEFAULT 0,
            tu REAL DEFAULT 0,
            method TEXT DEFAULT 'tyreus-luyben',
            tuned_at REAL NOT NULL
        )
    """)

    # approach_zones defines the multi-phase approach control profile per setpoint.
    # All offset values stored in Celsius deltas.
    # full_power_offset_c  — run at 100% until this many °C below setpoint
    # ramp_start_offset_c  — begin linear ramp from 100% down to cruise_power
    #                        over the span between full_power_offset and ramp_start_offset
    # cruise_power         — hold at this % output until pid_offset_c below setpoint
    # pid_offset_c         — hand off to PID at this many °C below setpoint
    c.execute("""
        CREATE TABLE IF NOT EXISTS approach_zones (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            setpoint_c REAL NOT NULL UNIQUE,
            full_power_offset_c REAL NOT NULL,
            ramp_start_offset_c REAL NOT NULL,
            cruise_power        REAL NOT NULL,
            pid_offset_c        REAL NOT NULL,
            created_at          REAL NOT NULL
        )
    """)

    # door_recovery_zones defines how the controller recovers after a door-open event.
    # boost_pct        — output % to run at immediately after door closes
    # resume_offset_c  — degrees C below setpoint at which recovery ends and
    #                    PID takes over, seeded from thermal model.
    # duration_s/taper_s kept for backward compatibility only — no longer used.
    c.execute("""
        CREATE TABLE IF NOT EXISTS door_recovery_zones (
            id               INTEGER PRIMARY KEY AUTOINCREMENT,
            setpoint_c       REAL NOT NULL UNIQUE,
            boost_pct        REAL NOT NULL,
            duration_s       REAL NOT NULL DEFAULT 0,
            taper_s          REAL NOT NULL DEFAULT 0,
            resume_offset_c  REAL NOT NULL DEFAULT 5.6,
            created_at       REAL NOT NULL
        )
    """)

    # ramp_tune_zones stores the identified process model and rate-controller gains
    # from a ramp step-response autotune, keyed by the oven temperature at which
    # the test was run.  The controller interpolates kp/ki between zones just like
    # PID zones, so you can tune at multiple temperatures for accuracy.
    #
    # center_temp_c  — oven temperature during the step test
    # kp             — rate-controller proportional gain
    # ki             — rate-controller integral gain
    # K_proc         — measured process gain (°C/min per % output)
    # L_s            — measured lag time (seconds)
    # step_output    — output % used during the step test
    c.execute("""
        CREATE TABLE IF NOT EXISTS ramp_tune_zones (
            id            INTEGER PRIMARY KEY AUTOINCREMENT,
            center_temp_c REAL NOT NULL UNIQUE,
            kp            REAL NOT NULL,
            ki            REAL NOT NULL,
            K_proc        REAL NOT NULL DEFAULT 0,
            L_s           REAL NOT NULL DEFAULT 0,
            step_output   REAL NOT NULL DEFAULT 50,
            tuned_at      REAL NOT NULL
        )
    """)

    c.execute("CREATE INDEX IF NOT EXISTS idx_temp_log_ts ON temperature_log(ts)")
    conn.commit()

    # Seed built-in programs if empty
    count = c.execute("SELECT COUNT(*) FROM programs").fetchone()[0]
    if count == 0:
        for prog in BUILTIN_PROGRAMS:
            _insert_program(conn, prog)
        logger.info(f"Seeded {len(BUILTIN_PROGRAMS)} built-in programs")

    # Seed default settings
    defaults = {
        "temp_unit":                  "F",
        "max_temp_f":                 "2100",   # User-facing max in °F
        "autotune_method":            "steelhaus",
        "approach_control_enabled":   "1",      # 1 = on, 0 = off
        "door_recovery_enabled":      "1",      # 1 = on, 0 = off
        "autotune_cruise_multiplier": "1.0",    # Scales preheat cruise output for weak ovens
        "autotune_relay_multiplier":  "1.0",    # Scales relay high output for weak ovens
    }
    for key, val in defaults.items():
        if not get_setting(conn, key):
            set_setting(conn, key, val)

    conn.close()
    logger.info("Database initialized")


# ── Programs ──────────────────────────────────────────────────────────────────

def _insert_program(conn, prog):
    segs = json.dumps([s.to_dict() for s in prog.segments])
    now = time.time()
    c = conn.cursor()
    c.execute(
        "INSERT INTO programs (name, description, segments_json, created_at, updated_at) VALUES (?,?,?,?,?)",
        (prog.name, prog.description, segs, now, now)
    )
    conn.commit()
    return c.lastrowid


def save_program(conn, prog):
    segs = json.dumps([s.to_dict() for s in prog.segments])
    now = time.time()
    if prog.id is None:
        c = conn.cursor()
        c.execute(
            "INSERT INTO programs (name, description, segments_json, created_at, updated_at) VALUES (?,?,?,?,?)",
            (prog.name, prog.description, segs, now, now)
        )
        conn.commit()
        prog.id = c.lastrowid
    else:
        conn.execute(
            "UPDATE programs SET name=?, description=?, segments_json=?, updated_at=? WHERE id=?",
            (prog.name, prog.description, segs, now, prog.id)
        )
        conn.commit()
    return prog


def delete_program(conn, program_id):
    conn.execute("DELETE FROM programs WHERE id=?", (program_id,))
    conn.commit()


def get_program(conn, program_id):
    row = conn.execute("SELECT * FROM programs WHERE id=?", (program_id,)).fetchone()
    return _row_to_program(row) if row else None


def get_all_programs(conn):
    rows = conn.execute("SELECT * FROM programs ORDER BY name").fetchall()
    return [_row_to_program(r) for r in rows]


def _row_to_program(row):
    segs = [Segment.from_dict(s) for s in json.loads(row["segments_json"])]
    return Program(
        id=row["id"], name=row["name"], description=row["description"],
        segments=segs, created_at=row["created_at"], updated_at=row["updated_at"],
    )


# ── Settings ──────────────────────────────────────────────────────────────────

def get_setting(conn, key, default=None):
    row = conn.execute("SELECT value FROM settings WHERE key=?", (key,)).fetchone()
    return row["value"] if row else default


def set_setting(conn, key, value):
    conn.execute(
        "INSERT INTO settings (key, value) VALUES (?,?) ON CONFLICT(key) DO UPDATE SET value=excluded.value",
        (key, str(value))
    )
    conn.commit()


def get_max_temp_c(conn) -> float:
    """Return the user-configured max temperature converted to Celsius."""
    unit = get_setting(conn, "temp_unit", "F")
    if unit == "F":
        max_f = float(get_setting(conn, "max_temp_f", "2100"))
        return (max_f - 32) * 5.0 / 9.0
    else:
        return float(get_setting(conn, "max_temp_f", "1149"))  # 2100F in C


# ── PID Zones ─────────────────────────────────────────────────────────────────

def save_pid_zone(conn, center_temp_c: float, kp: float, ki: float, kd: float,
                  ku: float = 0, tu: float = 0, method: str = "tyreus-luyben"):
    """Save or update PID gains for a temperature zone."""
    conn.execute("""
        INSERT INTO pid_zones (center_temp_c, kp, ki, kd, ku, tu, method, tuned_at)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?)
        ON CONFLICT(center_temp_c) DO UPDATE SET
            kp=excluded.kp, ki=excluded.ki, kd=excluded.kd,
            ku=excluded.ku, tu=excluded.tu, method=excluded.method,
            tuned_at=excluded.tuned_at
    """, (round(center_temp_c, 1), kp, ki, kd, ku, tu, method, time.time()))
    conn.commit()
    logger.info(f"Saved PID zone at {center_temp_c:.0f}°C: kp={kp}, ki={ki}, kd={kd}")


def delete_pid_zone(conn, zone_id: int):
    conn.execute("DELETE FROM pid_zones WHERE id=?", (zone_id,))
    conn.commit()


def get_all_pid_zones(conn) -> List[dict]:
    """Return all PID zones sorted by temperature."""
    rows = conn.execute(
        "SELECT * FROM pid_zones ORDER BY center_temp_c ASC"
    ).fetchall()
    return [dict(r) for r in rows]


def get_pid_params_for_temp(conn, temp_c: float) -> PIDParams:
    """
    Return interpolated PID params for the given temperature.

    Logic:
    - If no zones are stored, return safe conservative defaults.
    - If only one zone exists, use it regardless of temperature.
    - If temp is below the lowest zone, use the lowest zone's params.
    - If temp is above the highest zone, use the highest zone's params.
    - If temp falls between two zones, linearly interpolate Kp, Ki, Kd.

    Linear interpolation ensures smooth transitions with no sudden gain jumps
    as the oven heats up through zone boundaries.
    """
    zones = get_all_pid_zones(conn)

    if not zones:
        # No zones tuned yet — return safe conservative defaults
        return PIDParams(kp=1.0, ki=0.02, kd=0.5, output_min=0, output_max=100, sample_time=1.0)

    if len(zones) == 1:
        z = zones[0]
        return PIDParams(kp=z["kp"], ki=z["ki"], kd=z["kd"],
                         output_min=0, output_max=100, sample_time=1.0)

    # Below lowest zone
    if temp_c <= zones[0]["center_temp_c"]:
        z = zones[0]
        return PIDParams(kp=z["kp"], ki=z["ki"], kd=z["kd"],
                         output_min=0, output_max=100, sample_time=1.0)

    # Above highest zone
    if temp_c >= zones[-1]["center_temp_c"]:
        z = zones[-1]
        return PIDParams(kp=z["kp"], ki=z["ki"], kd=z["kd"],
                         output_min=0, output_max=100, sample_time=1.0)

    # Find surrounding zones and interpolate
    for i in range(len(zones) - 1):
        lo = zones[i]
        hi = zones[i + 1]
        if lo["center_temp_c"] <= temp_c <= hi["center_temp_c"]:
            span = hi["center_temp_c"] - lo["center_temp_c"]
            t = (temp_c - lo["center_temp_c"]) / span  # 0.0 to 1.0
            kp = lo["kp"] + t * (hi["kp"] - lo["kp"])
            ki = lo["ki"] + t * (hi["ki"] - lo["ki"])
            kd = lo["kd"] + t * (hi["kd"] - lo["kd"])
            return PIDParams(kp=round(kp, 6), ki=round(ki, 6), kd=round(kd, 6),
                             output_min=0, output_max=100, sample_time=1.0)

    # Fallback (shouldn't reach here)
    z = zones[-1]
    return PIDParams(kp=z["kp"], ki=z["ki"], kd=z["kd"],
                     output_min=0, output_max=100, sample_time=1.0)


# ── Approach Zones ────────────────────────────────────────────────────────────

def save_approach_zone(conn, setpoint_c: float, full_power_offset_c: float,
                       ramp_start_offset_c: float, cruise_power: float,
                       pid_offset_c: float):
    """Save or update an approach control zone."""
    conn.execute("""
        INSERT INTO approach_zones
            (setpoint_c, full_power_offset_c, ramp_start_offset_c, cruise_power, pid_offset_c, created_at)
        VALUES (?, ?, ?, ?, ?, ?)
        ON CONFLICT(setpoint_c) DO UPDATE SET
            full_power_offset_c=excluded.full_power_offset_c,
            ramp_start_offset_c=excluded.ramp_start_offset_c,
            cruise_power=excluded.cruise_power,
            pid_offset_c=excluded.pid_offset_c,
            created_at=excluded.created_at
    """, (round(setpoint_c, 1), full_power_offset_c, ramp_start_offset_c,
          cruise_power, pid_offset_c, time.time()))
    conn.commit()
    logger.info(f"Saved approach zone at {setpoint_c:.0f}°C")


def delete_approach_zone(conn, zone_id: int):
    conn.execute("DELETE FROM approach_zones WHERE id=?", (zone_id,))
    conn.commit()


def get_all_approach_zones(conn) -> List[dict]:
    """Return all approach zones sorted by setpoint temperature."""
    rows = conn.execute(
        "SELECT * FROM approach_zones ORDER BY setpoint_c ASC"
    ).fetchall()
    return [dict(r) for r in rows]


def get_approach_params_for_temp(conn, setpoint_c: float) -> Optional[dict]:
    """
    Return interpolated approach params for the given setpoint.

    Returns a dict with keys:
        full_power_offset_c, ramp_start_offset_c, cruise_power, pid_offset_c

    Interpolation logic mirrors get_pid_params_for_temp:
    - No zones: return None (controller falls back to simple 100°F threshold)
    - One zone: use it for all setpoints
    - Below lowest zone: use lowest zone
    - Above highest zone: use highest zone
    - Between zones: linearly interpolate all four params
    """
    zones = get_all_approach_zones(conn)

    if not zones:
        return None

    def _zone_to_dict(z):
        return {
            "full_power_offset_c": z["full_power_offset_c"],
            "ramp_start_offset_c": z["ramp_start_offset_c"],
            "cruise_power":        z["cruise_power"],
            "pid_offset_c":        z["pid_offset_c"],
        }

    if len(zones) == 1:
        return _zone_to_dict(zones[0])

    if setpoint_c <= zones[0]["setpoint_c"]:
        return _zone_to_dict(zones[0])

    if setpoint_c >= zones[-1]["setpoint_c"]:
        return _zone_to_dict(zones[-1])

    for i in range(len(zones) - 1):
        lo = zones[i]
        hi = zones[i + 1]
        if lo["setpoint_c"] <= setpoint_c <= hi["setpoint_c"]:
            span = hi["setpoint_c"] - lo["setpoint_c"]
            t = (setpoint_c - lo["setpoint_c"]) / span
            def interp(key):
                return lo[key] + t * (hi[key] - lo[key])
            return {
                "full_power_offset_c": interp("full_power_offset_c"),
                "ramp_start_offset_c": interp("ramp_start_offset_c"),
                "cruise_power":        interp("cruise_power"),
                "pid_offset_c":        interp("pid_offset_c"),
            }

    return _zone_to_dict(zones[-1])


# ── Door Recovery Zones ───────────────────────────────────────────────────────

def save_door_recovery_zone(conn, setpoint_c: float, boost_pct: float,
                            resume_offset_c: float = 5.6):
    """Save or replace a door recovery zone.
    boost_pct        -- output % to apply during recovery
    resume_offset_c  -- degrees C below setpoint at which to hand off to PID
    """
    conn.execute("""
        INSERT INTO door_recovery_zones
            (setpoint_c, boost_pct, duration_s, taper_s, resume_offset_c, created_at)
        VALUES (?,?,0,0,?,?)
        ON CONFLICT(setpoint_c) DO UPDATE SET
            boost_pct=excluded.boost_pct,
            resume_offset_c=excluded.resume_offset_c,
            created_at=excluded.created_at
    """, (round(setpoint_c, 1), boost_pct, resume_offset_c, time.time()))
    conn.commit()
    logger.info(
        f"Door recovery zone saved at {setpoint_c:.0f}\u00b0C: "
        f"boost={boost_pct:.0f}%, resume at -{resume_offset_c:.1f}\u00b0C"
    )

def delete_door_recovery_zone(conn, zone_id: int):
    conn.execute("DELETE FROM door_recovery_zones WHERE id=?", (zone_id,))
    conn.commit()


def get_all_door_recovery_zones(conn) -> List[dict]:
    """Return all door recovery zones sorted by setpoint temperature."""
    rows = conn.execute(
        "SELECT id, setpoint_c, boost_pct, "
        "COALESCE(resume_offset_c, 5.6) as resume_offset_c, created_at "
        "FROM door_recovery_zones ORDER BY setpoint_c ASC"
    ).fetchall()
    return [{"id": r[0], "setpoint_c": r[1], "boost_pct": r[2],
             "resume_offset_c": r[3], "created_at": r[4]}
            for r in rows]


def get_door_recovery_params(conn, setpoint_c: float) -> Optional[dict]:
    """
    Return interpolated door recovery params for the given setpoint.
    Returns a dict with keys: boost_pct, resume_offset_c
    Returns None if no zones are defined (controller skips recovery).
    """
    zones = get_all_door_recovery_zones(conn)

    if not zones:
        return None

    def _to_dict(z):
        return {
            "boost_pct":       z["boost_pct"],
            "resume_offset_c": z["resume_offset_c"],
        }

    if len(zones) == 1:
        return _to_dict(zones[0])

    if setpoint_c <= zones[0]["setpoint_c"]:
        return _to_dict(zones[0])

    if setpoint_c >= zones[-1]["setpoint_c"]:
        return _to_dict(zones[-1])

    for i in range(len(zones) - 1):
        lo = zones[i]
        hi = zones[i + 1]
        if lo["setpoint_c"] <= setpoint_c <= hi["setpoint_c"]:
            span = hi["setpoint_c"] - lo["setpoint_c"]
            t = (setpoint_c - lo["setpoint_c"]) / span
            return {
                "boost_pct":       lo["boost_pct"] + t * (hi["boost_pct"] - lo["boost_pct"]),
                "resume_offset_c": lo["resume_offset_c"] + t * (hi["resume_offset_c"] - lo["resume_offset_c"]),
            }

    return _to_dict(zones[-1])


# ── Sessions ──────────────────────────────────────────────────────────────────

def _make_session_label(ts: float) -> str:
    """Human-readable label like 'Jun 03 2026  14:32'."""
    import datetime
    dt = datetime.datetime.fromtimestamp(ts)
    return dt.strftime("%b %d %Y  %H:%M")


def new_session(conn) -> int:
    """Close any open sessions and open a fresh one. Returns the new session id."""
    now = time.time()
    # Close all open sessions
    conn.execute("UPDATE sessions SET ended_at=? WHERE ended_at IS NULL", (now,))
    label = _make_session_label(now)
    c = conn.cursor()
    c.execute("INSERT INTO sessions (label, started_at) VALUES (?,?)", (label, now))
    conn.commit()
    session_id = c.lastrowid
    logger.info(f"New session #{session_id} started: {label}")
    return session_id


def close_session(conn, session_id: int):
    """Stamp ended_at on the given session (called on shutdown)."""
    conn.execute(
        "UPDATE sessions SET ended_at=? WHERE id=? AND ended_at IS NULL",
        (time.time(), session_id)
    )
    conn.commit()
    logger.info(f"Session #{session_id} closed")


def get_all_sessions(conn) -> List[dict]:
    """Return all sessions newest-first."""
    rows = conn.execute(
        "SELECT id, label, started_at, ended_at, COALESCE(protected,0) as protected "
        "FROM sessions ORDER BY started_at DESC"
    ).fetchall()
    return [{"id": r[0], "label": r[1], "started_at": r[2],
             "ended_at": r[3], "protected": bool(r[4])} for r in rows]


def set_session_protected(conn, session_id: int, protected: bool):
    """Set or clear the protected flag on a session."""
    conn.execute(
        "UPDATE sessions SET protected=? WHERE id=?",
        (1 if protected else 0, session_id)
    )
    conn.commit()
    logger.info(f"Session #{session_id} protected={protected}")


def delete_session(conn, session_id: int):
    """Delete a session and all its temperature log rows.
    Refuses to delete protected sessions."""
    row = conn.execute(
        "SELECT COALESCE(protected,0) FROM sessions WHERE id=?", (session_id,)
    ).fetchone()
    if row and row[0]:
        logger.warning(f"delete_session: session #{session_id} is protected — skipping")
        return
    conn.execute("DELETE FROM temperature_log WHERE session_id=?", (session_id,))
    conn.execute("DELETE FROM sessions WHERE id=?", (session_id,))
    conn.commit()
    logger.info(f"Session #{session_id} deleted")


def get_current_session_id(conn) -> Optional[int]:
    """Return the id of the currently open session, or None."""
    row = conn.execute(
        "SELECT id FROM sessions WHERE ended_at IS NULL ORDER BY started_at DESC LIMIT 1"
    ).fetchone()
    return row["id"] if row else None


# ── Temperature Log ───────────────────────────────────────────────────────────

def log_temperature(conn, ts, temp, setpoint, pid_output, program_name="", segment_index=-1, session_id=1, door_open=0):
    conn.execute(
        "INSERT INTO temperature_log (ts, temp, setpoint, pid_output, program_name, segment_index, session_id, door_open) VALUES (?,?,?,?,?,?,?,?)",
        (ts, round(temp, 2), round(setpoint, 2), round(pid_output, 2), program_name, segment_index, session_id, int(door_open))
    )
    conn.commit()


def get_temperature_log(conn, since_ts=None, limit=10000, session_id=None):
    if session_id is not None:
        if since_ts:
            rows = conn.execute(
                "SELECT ts, temp, setpoint, pid_output, COALESCE(door_open,0) AS door_open FROM temperature_log WHERE session_id=? AND ts>=? ORDER BY ts DESC LIMIT ?",
                (session_id, since_ts, limit)
            ).fetchall()
        else:
            rows = conn.execute(
                "SELECT ts, temp, setpoint, pid_output, COALESCE(door_open,0) AS door_open FROM temperature_log WHERE session_id=? ORDER BY ts DESC LIMIT ?",
                (session_id, limit)
            ).fetchall()
    elif since_ts:
        rows = conn.execute(
            "SELECT ts, temp, setpoint, pid_output, COALESCE(door_open,0) AS door_open FROM temperature_log WHERE ts >= ? ORDER BY ts DESC LIMIT ?",
            (since_ts, limit)
        ).fetchall()
    else:
        rows = conn.execute(
            "SELECT ts, temp, setpoint, pid_output, COALESCE(door_open,0) AS door_open FROM temperature_log ORDER BY ts DESC LIMIT ?",
            (limit,)
        ).fetchall()
    return [dict(r) for r in reversed(rows)]


def purge_old_logs(conn, days=30):
    """Delete sessions and their log data older than `days` days.
    Protected sessions are never deleted regardless of age."""
    cutoff = time.time() - days * 86400
    # Delete log rows for non-protected sessions older than cutoff
    conn.execute(
        "DELETE FROM temperature_log WHERE session_id IN ("
        "  SELECT id FROM sessions WHERE ended_at IS NOT NULL "
        "  AND ended_at < ? AND COALESCE(protected,0)=0"
        ")", (cutoff,)
    )
    # Remove those now-empty sessions
    conn.execute(
        "DELETE FROM sessions WHERE ended_at IS NOT NULL AND ended_at < ? "
        "AND COALESCE(protected,0)=0 "
        "AND id NOT IN (SELECT DISTINCT session_id FROM temperature_log)",
        (cutoff,)
    )
    conn.commit()
    logger.info(f"purge_old_logs: removed unprotected data older than {days} days")


def bulk_delete_sessions(conn, session_ids: list, current_session_id=None) -> int:
    """Delete a list of sessions by id, skipping the active or protected sessions.
    Returns the number of sessions actually deleted."""
    if not session_ids:
        return 0
    deleted = 0
    for sid in session_ids:
        if current_session_id is not None and sid == current_session_id:
            continue   # never delete the active session
        row = conn.execute(
            "SELECT COALESCE(protected,0) FROM sessions WHERE id=?", (sid,)
        ).fetchone()
        if row and row[0]:
            continue   # skip protected sessions
        conn.execute("DELETE FROM temperature_log WHERE session_id=?", (sid,))
        conn.execute("DELETE FROM sessions WHERE id=?", (sid,))
        deleted += 1
    conn.commit()
    logger.info(f"bulk_delete_sessions: deleted {deleted} session(s)")
    return deleted


def get_session_stats(conn) -> dict:
    """Return session count and approximate total log row count for the UI."""
    total_sessions = conn.execute(
        "SELECT COUNT(*) FROM sessions WHERE ended_at IS NOT NULL"
    ).fetchone()[0]
    total_rows = conn.execute(
        "SELECT COUNT(*) FROM temperature_log"
    ).fetchone()[0]
    oldest = conn.execute(
        "SELECT MIN(started_at) FROM sessions WHERE ended_at IS NOT NULL"
    ).fetchone()[0]
    return {
        "total_sessions": total_sessions,
        "total_log_rows": total_rows,
        "oldest_session_ts": oldest,
    }


# ── Ramp Tune Zones ───────────────────────────────────────────────────────────

def save_ramp_tune_zone(conn, center_temp_c: float, kp: float, ki: float,
                        K_proc: float = 0.0, L_s: float = 0.0,
                        step_output: float = 50.0, hold_pct: float = 0.0):
    """Save or replace a ramp tune zone at the given temperature.
    hold_pct is the measured steady-state PID output needed to hold
    center_temp_c — captured during the stabilize phase of the step test."""
    conn.execute("""
        INSERT INTO ramp_tune_zones
            (center_temp_c, kp, ki, K_proc, L_s, step_output, hold_pct, tuned_at)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?)
        ON CONFLICT(center_temp_c) DO UPDATE SET
            kp=excluded.kp, ki=excluded.ki,
            K_proc=excluded.K_proc, L_s=excluded.L_s,
            step_output=excluded.step_output,
            hold_pct=excluded.hold_pct,
            tuned_at=excluded.tuned_at
    """, (round(center_temp_c, 1), kp, ki, K_proc, L_s, step_output, hold_pct, time.time()))
    conn.commit()


def delete_ramp_tune_zone(conn, zone_id: int):
    conn.execute("DELETE FROM ramp_tune_zones WHERE id = ?", (zone_id,))
    conn.commit()


def get_all_ramp_tune_zones(conn) -> list:
    rows = conn.execute(
        "SELECT id, center_temp_c, kp, ki, K_proc, L_s, step_output, "
        "COALESCE(hold_pct,0) as hold_pct, tuned_at "
        "FROM ramp_tune_zones ORDER BY center_temp_c"
    ).fetchall()
    return [
        {"id": r[0], "center_temp_c": r[1], "kp": r[2], "ki": r[3],
         "K_proc": r[4], "L_s": r[5], "step_output": r[6],
         "hold_pct": r[7], "tuned_at": r[8]}
        for r in rows
    ]


def get_ramp_tune_params(conn, temp_c: float) -> dict:
    """
    Return interpolated ramp rate-controller gains for the given temperature.
    Interpolates between zones exactly like get_pid_params_for_temp.
    Returns default gains if no zones exist.
    """
    zones = get_all_ramp_tune_zones(conn)

    if not zones:
        return {"kp": 2.0, "ki": 0.10, "K_proc": 0.0, "hold_pct": 0.0}

    if len(zones) == 1:
        z = zones[0]
        return {"kp": z["kp"], "ki": z["ki"], "K_proc": z["K_proc"],
                "hold_pct": z["hold_pct"]}

    if temp_c <= zones[0]["center_temp_c"]:
        z = zones[0]
        return {"kp": z["kp"], "ki": z["ki"], "K_proc": z["K_proc"],
                "hold_pct": z["hold_pct"]}

    if temp_c >= zones[-1]["center_temp_c"]:
        z = zones[-1]
        return {"kp": z["kp"], "ki": z["ki"], "K_proc": z["K_proc"],
                "hold_pct": z["hold_pct"]}

    for i in range(len(zones) - 1):
        lo, hi = zones[i], zones[i + 1]
        if lo["center_temp_c"] <= temp_c <= hi["center_temp_c"]:
            span   = hi["center_temp_c"] - lo["center_temp_c"]
            t      = (temp_c - lo["center_temp_c"]) / span
            kp     = lo["kp"]      + t * (hi["kp"]      - lo["kp"])
            ki     = lo["ki"]      + t * (hi["ki"]      - lo["ki"])
            K_proc = lo["K_proc"]  + t * (hi["K_proc"]  - lo["K_proc"])
            hold   = lo["hold_pct"]+ t * (hi["hold_pct"]- lo["hold_pct"])
            return {"kp": round(kp, 4), "ki": round(ki, 6),
                    "K_proc": round(K_proc, 6), "hold_pct": round(hold, 2)}

    z = zones[-1]
    return {"kp": z["kp"], "ki": z["ki"], "K_proc": z["K_proc"],
            "hold_pct": z["hold_pct"]}


# ── Thermal Model ─────────────────────────────────────────────────────────────
# Continuously-refined hold-power curve.  Anchor points are spaced
# MODEL_RESOLUTION_C apart.  Each steady-state observation updates the
# nearest anchor via exponential moving average with EMA_ALPHA weight.

MODEL_RESOLUTION_C = 5.0    # °C between anchor points
EMA_ALPHA          = 0.10   # weight of each new sample (0.1 = 10% new, 90% old)


def _model_anchor(temp_c: float) -> float:
    """Round temp_c to the nearest MODEL_RESOLUTION_C anchor."""
    return round(round(temp_c / MODEL_RESOLUTION_C) * MODEL_RESOLUTION_C, 1)


def update_thermal_model(conn, temp_c: float, hold_pct: float):
    """
    Record a steady-state observation at temp_c with hold_pct output.
    Updates the nearest anchor point using exponential moving average.
    Creates the anchor if it doesn't exist yet.
    """
    anchor = _model_anchor(temp_c)
    now    = time.time()
    row    = conn.execute(
        "SELECT hold_pct, samples FROM thermal_model WHERE temp_c=?", (anchor,)
    ).fetchone()

    if row is None:
        # First observation at this anchor — bootstrap with this value
        conn.execute(
            "INSERT INTO thermal_model (temp_c, hold_pct, samples, updated_at) "
            "VALUES (?,?,1,?)",
            (anchor, round(hold_pct, 2), now)
        )
    else:
        old_pct, samples = row
        # EMA: blend new observation in with decreasing weight as samples grow.
        # Use a floor alpha so the model stays responsive to long-term drift.
        alpha   = max(EMA_ALPHA, 1.0 / (samples + 1))
        new_pct = old_pct * (1.0 - alpha) + hold_pct * alpha
        conn.execute(
            "UPDATE thermal_model SET hold_pct=?, samples=?, updated_at=? "
            "WHERE temp_c=?",
            (round(new_pct, 2), samples + 1, now, anchor)
        )
    conn.commit()


def get_hold_estimate(conn, temp_c: float) -> Optional[float]:
    """
    Return the best estimate of hold power (%) needed to maintain temp_c.
    Interpolates between the two nearest anchor points.
    Returns None if insufficient model data exists (< 2 points).
    """
    rows = conn.execute(
        "SELECT temp_c, hold_pct FROM thermal_model ORDER BY temp_c"
    ).fetchall()

    if not rows:
        return None

    points = [(r[0], r[1]) for r in rows]

    if len(points) == 1:
        # Only one anchor — use it directly if close enough
        if abs(points[0][0] - temp_c) <= MODEL_RESOLUTION_C * 3:
            return points[0][1]
        return None

    # Below lowest anchor
    if temp_c <= points[0][0]:
        return points[0][1]

    # Above highest anchor
    if temp_c >= points[-1][0]:
        return points[-1][1]

    # Interpolate between surrounding anchors
    for i in range(len(points) - 1):
        lo_t, lo_p = points[i]
        hi_t, hi_p = points[i + 1]
        if lo_t <= temp_c <= hi_t:
            t = (temp_c - lo_t) / (hi_t - lo_t)
            return round(lo_p + t * (hi_p - lo_p), 2)

    return points[-1][1]


def get_all_thermal_model(conn) -> list:
    """Return all thermal model anchor points ordered by temperature."""
    rows = conn.execute(
        "SELECT id, temp_c, hold_pct, samples, updated_at "
        "FROM thermal_model ORDER BY temp_c"
    ).fetchall()
    return [{"id": r[0], "temp_c": r[1], "hold_pct": r[2],
             "samples": r[3], "updated_at": r[4]} for r in rows]


def clear_thermal_model(conn):
    """Wipe all thermal model data (use if oven is reconfigured significantly)."""
    conn.execute("DELETE FROM thermal_model")
    conn.commit()
    logger.info("Thermal model cleared")
