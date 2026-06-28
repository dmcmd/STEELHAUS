"""
app.py — Flask web server for kiln controller.
"""

import os
import subprocess
import time
import json
import logging
import threading
import queue as queue_module

from flask import Flask, jsonify, request, Response, send_from_directory
from flask_cors import CORS

from controller import KilnController
from program_store import (
    get_db, get_all_programs, get_program, save_program, delete_program,
    get_temperature_log, get_setting, set_setting,
    save_pid_zone, get_all_pid_zones, delete_pid_zone, get_max_temp_c,
    save_approach_zone, get_all_approach_zones, delete_approach_zone,
    save_door_recovery_zone, get_all_door_recovery_zones, delete_door_recovery_zone,
    new_session, close_session, get_all_sessions, delete_session, get_current_session_id,
    bulk_delete_sessions, purge_old_logs, get_session_stats, set_session_protected,
    get_all_thermal_model, clear_thermal_model
)
from program_engine import Program, Segment, SegmentType

# ── Config ────────────────────────────────────────────────────────────────────

DB_PATH     = os.environ.get("KILN_DB",     "/home/pi/kiln_controller/data/kiln.db")
SERIAL_PORT = os.environ.get("KILN_SERIAL", "/dev/ttyUSB0")
HOST        = "0.0.0.0"
PORT        = int(os.environ.get("KILN_PORT", "5000"))

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s")
logger = logging.getLogger("app")

app  = Flask(__name__, static_folder="static")
CORS(app)

db   = get_db(DB_PATH)
kiln = KilnController(db_path=DB_PATH, serial_port=SERIAL_PORT)

# Create a new session every time the server starts (covers power cycles)
_current_session_id = new_session(db)
kiln.set_session_id(_current_session_id)

# Auto-purge old sessions on startup using the retention_days setting.
# Default 90 days — keeps plenty of history without unbounded growth.
try:
    from program_store import get_setting
    _retention = int(get_setting(db, "retention_days") or 90)
    if _retention > 0:
        purge_old_logs(db, days=_retention)
except Exception as _e:
    import logging
    logging.getLogger("app").warning(f"Auto-purge on startup failed: {_e}")

# ── SSE broadcast ─────────────────────────────────────────────────────────────

_sse_clients: list = []
_sse_lock = threading.Lock()
_sim_mode = False


def _broadcast_loop():
    while True:
        time.sleep(1.0)
        try:
            status = kiln.get_status()
            status["sim_mode"] = _sim_mode
            data   = f"data: {json.dumps(status)}\n\n"
            with _sse_lock:
                dead = []
                for q in _sse_clients:
                    try:
                        q.put_nowait(data)
                    except queue_module.Full:
                        dead.append(q)
                for q in dead:
                    _sse_clients.remove(q)
        except Exception as e:
            logger.error(f"SSE error: {e}")


threading.Thread(target=_broadcast_loop, daemon=True, name="sse").start()


@app.route("/api/stream")
def stream():
    q = queue_module.Queue(maxsize=10)
    with _sse_lock:
        _sse_clients.append(q)

    def generate():
        try:
            status = kiln.get_status()
            status["sim_mode"] = _sim_mode
            yield f"data: {json.dumps(status)}\n\n"
        except Exception:
            pass
        while True:
            try:
                yield q.get(timeout=30)
            except queue_module.Empty:
                yield ": keepalive\n\n"
            except GeneratorExit:
                break
        with _sse_lock:
            try:
                _sse_clients.remove(q)
            except ValueError:
                pass

    return Response(generate(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})


# ── Status & basic control ────────────────────────────────────────────────────

@app.route("/api/status")
def api_status():
    return jsonify(kiln.get_status())


@app.route("/api/control/enable", methods=["POST"])
def api_enable():
    data   = request.json or {}
    enable = bool(data.get("enable", True))
    kiln.enable_heater(enable)
    return jsonify({"ok": True, "heater_enabled": enable})


@app.route("/api/control/contactor", methods=["POST"])
def api_contactor():
    data = request.json or {}
    on   = bool(data.get("on", False))
    kiln._bridge.set_contactor(on)
    return jsonify({"ok": True, "contactor": on})


@app.route("/api/control/setpoint", methods=["POST"])
def api_setpoint():
    data   = request.json or {}
    temp_c = float(data["temp_c"])
    kiln.set_manual_setpoint(temp_c)
    return jsonify({"ok": True, "setpoint_c": temp_c})


@app.route("/api/control/manual_output", methods=["POST"])
def api_manual_output():
    data = request.json or {}
    pct  = data.get("pct")
    if pct is not None:
        pct = float(pct)
    kiln.set_manual_output(pct)
    return jsonify({"ok": True, "manual_output": pct})


@app.route("/api/control/estop", methods=["POST"])
def api_estop():
    kiln.emergency_stop()
    return jsonify({"ok": True})


@app.route("/api/control/reset_estop", methods=["POST"])
def api_reset_estop():
    kiln.reset_estop()
    return jsonify({"ok": True})


# ── PID (manual override) ─────────────────────────────────────────────────────

@app.route("/api/pid", methods=["POST"])
def api_pid_set():
    data = request.json or {}
    kiln.set_pid_params_manual(float(data["kp"]), float(data["ki"]), float(data["kd"]))
    return jsonify({"ok": True})


# ── PID Zones ─────────────────────────────────────────────────────────────────

@app.route("/api/pid/zones", methods=["GET"])
def api_zones_list():
    return jsonify(get_all_pid_zones(db))


@app.route("/api/pid/zones", methods=["POST"])
def api_zone_save():
    """Manually save or update a PID zone."""
    data = request.json or {}
    try:
        save_pid_zone(
            db,
            center_temp_c=float(data["center_temp_c"]),
            kp=float(data["kp"]),
            ki=float(data["ki"]),
            kd=float(data["kd"]),
        )
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 400


@app.route("/api/pid/zones/<int:zone_id>", methods=["DELETE"])
def api_zone_delete(zone_id):
    kiln.delete_pid_zone(zone_id)
    return jsonify({"ok": True})


# ── Approach Zones ────────────────────────────────────────────────────────────

@app.route("/api/approach/zones", methods=["GET"])
def api_approach_zones_list():
    return jsonify(get_all_approach_zones(db))


@app.route("/api/approach/zones", methods=["POST"])
def api_approach_zone_save():
    data = request.json or {}
    try:
        save_approach_zone(
            db,
            setpoint_c          = float(data["setpoint_c"]),
            full_power_offset_c = float(data["full_power_offset_c"]),
            ramp_start_offset_c = float(data["ramp_start_offset_c"]),
            cruise_power        = float(data["cruise_power"]),
            pid_offset_c        = float(data["pid_offset_c"]),
        )
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 400


@app.route("/api/approach/zones/<int:zone_id>", methods=["DELETE"])
def api_approach_zone_delete(zone_id):
    delete_approach_zone(db, zone_id)
    return jsonify({"ok": True})


# ── Door Recovery Zones ───────────────────────────────────────────────────────

@app.route("/api/door_recovery/zones", methods=["GET"])
def api_door_recovery_zones_list():
    return jsonify(get_all_door_recovery_zones(db))


@app.route("/api/door_recovery/zones", methods=["POST"])
def api_door_recovery_zone_save():
    data = request.json or {}
    try:
        setpoint_c      = float(data["setpoint_c"])
        boost_pct       = float(data["boost_pct"])
        resume_offset_c = float(data.get("resume_offset_c", 5.6))
        if any(v != v for v in [setpoint_c, boost_pct, resume_offset_c]):  # NaN check
            return jsonify({"ok": False, "error": "Invalid numeric value"}), 400
        save_door_recovery_zone(
            db,
            setpoint_c      = setpoint_c,
            boost_pct       = boost_pct,
            resume_offset_c = resume_offset_c,
        )
        return jsonify({"ok": True})
    except KeyError as e:
        return jsonify({"ok": False, "error": f"Missing field: {e}"}), 400
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 400


@app.route("/api/door_recovery/zones/<int:zone_id>", methods=["DELETE"])
def api_door_recovery_zone_delete(zone_id):
    delete_door_recovery_zone(db, zone_id)
    return jsonify({"ok": True})


# ── Autotune ──────────────────────────────────────────────────────────────────

@app.route("/api/autotune/start", methods=["POST"])
def api_autotune_start():
    data       = request.json or {}
    setpoint_c = float(data.get("setpoint_c", 427))   # default ~800°F
    method     = data.get("method", "steelhaus")
    kiln.start_autotune(setpoint_c=setpoint_c, method=method)
    return jsonify({"ok": True, "setpoint_c": setpoint_c, "method": method})


@app.route("/api/autotune/abort", methods=["POST"])
def api_autotune_abort():
    kiln.abort_autotune()
    return jsonify({"ok": True})


@app.route("/api/autotune/apply", methods=["POST"])
def api_autotune_apply():
    try:
        kiln.apply_autotune_result()
        return jsonify({"ok": True})
    except RuntimeError as e:
        return jsonify({"ok": False, "error": str(e)}), 400


# ── Ramp Autotune ─────────────────────────────────────────────────────────────

@app.route("/api/ramp_autotune/start", methods=["POST"])
def api_ramp_autotune_start():
    data             = request.json or {}
    stabilize_temp_c = float(data.get("stabilize_temp_c", 177))   # default ~350°F
    step_output      = float(data.get("step_output", 50.0))
    step_duration_s  = float(data.get("step_duration_s", 360.0))
    kiln.start_ramp_autotune(
        stabilize_temp_c=stabilize_temp_c,
        step_output=step_output,
        step_duration_s=step_duration_s,
    )
    return jsonify({"ok": True, "stabilize_temp_c": stabilize_temp_c,
                    "step_output": step_output, "step_duration_s": step_duration_s})


@app.route("/api/ramp_autotune/abort", methods=["POST"])
def api_ramp_autotune_abort():
    kiln.abort_ramp_autotune()
    return jsonify({"ok": True})


@app.route("/api/ramp_autotune/apply", methods=["POST"])
def api_ramp_autotune_apply():
    try:
        kiln.apply_ramp_autotune_result()
        return jsonify({"ok": True})
    except RuntimeError as e:
        return jsonify({"ok": False, "error": str(e)}), 400


@app.route("/api/ramp_tune/zones", methods=["GET"])
def api_ramp_tune_zones():
    from program_store import get_all_ramp_tune_zones
    return jsonify(get_all_ramp_tune_zones(db))


@app.route("/api/ramp_tune/zones/<int:zone_id>", methods=["DELETE"])
def api_ramp_tune_zone_delete(zone_id):
    kiln.delete_ramp_tune_zone(zone_id)
    return jsonify({"ok": True})


# ── Thermal Model ─────────────────────────────────────────────────────────────

@app.route("/api/thermal_model", methods=["GET"])
def api_thermal_model_get():
    return jsonify(get_all_thermal_model(db))


@app.route("/api/thermal_model/clear", methods=["POST"])
def api_thermal_model_clear():
    clear_thermal_model(db)
    return jsonify({"ok": True})


# ── Programs ──────────────────────────────────────────────────────────────────

@app.route("/api/programs", methods=["GET"])
def api_programs_list():
    return jsonify([p.to_dict() for p in get_all_programs(db)])


@app.route("/api/programs/<int:pid>", methods=["GET"])
def api_program_get(pid):
    prog = get_program(db, pid)
    return jsonify(prog.to_dict()) if prog else (jsonify({"error": "Not found"}), 404)


@app.route("/api/programs", methods=["POST"])
def api_program_create():
    try:
        prog = Program.from_dict(request.json or {})
        prog.id = None
        return jsonify(save_program(db, prog).to_dict()), 201
    except Exception as e:
        return jsonify({"error": str(e)}), 400


@app.route("/api/programs/<int:pid>", methods=["PUT"])
def api_program_update(pid):
    try:
        prog    = Program.from_dict(request.json or {})
        prog.id = pid
        return jsonify(save_program(db, prog).to_dict())
    except Exception as e:
        return jsonify({"error": str(e)}), 400


@app.route("/api/programs/<int:pid>", methods=["DELETE"])
def api_program_delete(pid):
    delete_program(db, pid)
    return jsonify({"ok": True})


@app.route("/api/programs/<int:pid>/start", methods=["POST"])
def api_program_start(pid):
    prog = get_program(db, pid)
    if not prog:
        return jsonify({"error": "Not found"}), 404
    kiln.load_program(prog)
    kiln.start_program()
    return jsonify({"ok": True})


@app.route("/api/program/pause",  methods=["POST"])
def api_program_pause():
    kiln.pause_program()
    return jsonify({"ok": True})


@app.route("/api/program/resume", methods=["POST"])
def api_program_resume():
    kiln.resume_program()
    return jsonify({"ok": True})


@app.route("/api/program/abort",  methods=["POST"])
def api_program_abort():
    kiln.abort_program()
    return jsonify({"ok": True})


# ── Temperature log ───────────────────────────────────────────────────────────

@app.route("/api/log")
def api_log():
    since      = request.args.get("since", type=float)
    limit      = request.args.get("limit", 10000, type=int)
    session_id = request.args.get("session_id", type=int)
    # Default: return the current session only
    if session_id is None:
        session_id = _current_session_id
    return jsonify(get_temperature_log(db, since_ts=since, limit=limit, session_id=session_id))


# ── Sessions ──────────────────────────────────────────────────────────────────

@app.route("/api/sessions", methods=["GET"])
def api_sessions_list():
    return jsonify(get_all_sessions(db))


@app.route("/api/sessions/new", methods=["POST"])
def api_session_new():
    """Close current session and start a fresh one (without shutting down)."""
    global _current_session_id
    _current_session_id = new_session(db)
    kiln.set_session_id(_current_session_id)
    return jsonify({"ok": True, "session_id": _current_session_id})


@app.route("/api/sessions/<int:session_id>", methods=["DELETE"])
def api_session_delete(session_id):
    if session_id == _current_session_id:
        return jsonify({"ok": False, "error": "Cannot delete the active session"}), 400
    delete_session(db, session_id)
    return jsonify({"ok": True})


@app.route("/api/sessions/<int:session_id>", methods=["PATCH"])
def api_session_patch(session_id):
    data = request.json or {}
    if "protected" in data:
        set_session_protected(db, session_id, bool(data["protected"]))
    return jsonify({"ok": True})


@app.route("/api/sessions/bulk_delete", methods=["POST"])
def api_sessions_bulk_delete():
    data = request.json or {}
    ids  = data.get("ids", [])
    if not isinstance(ids, list):
        return jsonify({"ok": False, "error": "ids must be a list"}), 400
    ids = [int(i) for i in ids if str(i).isdigit()]
    deleted = bulk_delete_sessions(db, ids, current_session_id=_current_session_id)
    return jsonify({"ok": True, "deleted": deleted})


@app.route("/api/sessions/purge", methods=["POST"])
def api_sessions_purge():
    data = request.json or {}
    days = int(data.get("days", 30))
    days = max(1, days)
    purge_old_logs(db, days=days)
    return jsonify({"ok": True, "days": days})


@app.route("/api/sessions/stats", methods=["GET"])
def api_sessions_stats():
    return jsonify(get_session_stats(db))


# ── Settings ──────────────────────────────────────────────────────────────────

@app.route("/api/settings", methods=["GET"])
def api_settings_get():
    keys = ["temp_unit", "max_temp_f", "autotune_method", "temp_offset_c",
            "approach_control_enabled", "door_recovery_enabled",
            "autotune_cruise_multiplier", "autotune_relay_multiplier"]
    return jsonify({k: get_setting(db, k) for k in keys})


@app.route("/api/settings", methods=["POST"])
def api_settings_set():
    data = request.json or {}
    for k, v in data.items():
        set_setting(db, k, str(v))
    kiln.reload_max_temp()
    if "temp_offset_c" in data:
        kiln.reload_temp_offset()
    if "approach_control_enabled" in data:
        kiln.reload_approach_control(bool(int(data["approach_control_enabled"])))
    if "door_recovery_enabled" in data:
        kiln.reload_door_recovery(bool(int(data["door_recovery_enabled"])))
    if "autotune_cruise_multiplier" in data:
        kiln.reload_cruise_multiplier(max(0.1, float(data["autotune_cruise_multiplier"])))
    if "autotune_relay_multiplier" in data:
        kiln.reload_relay_multiplier(max(0.1, float(data["autotune_relay_multiplier"])))
    return jsonify({"ok": True})


# ── Sim mode ──────────────────────────────────────────────────────────────────

@app.route("/api/sim", methods=["POST"])
def api_sim():
    global _sim_mode
    data = request.json or {}
    _sim_mode = bool(data.get("sim_mode", False))
    return jsonify({"ok": True, "sim_mode": _sim_mode})


# ── Shutdown ─────────────────────────────────────────────────────────────────

# ── WiFi (NetworkManager / nmcli — Bookworm default) ─────────────────────────

def _nm_wifi_profiles():
    """Return a dict mapping profile_name -> {ssid, priority} for all saved WiFi profiles."""
    list_result = subprocess.run(
        ["nmcli", "--escape", "no", "-t", "-f", "TYPE,NAME", "connection", "show"],
        capture_output=True, text=True, timeout=5
    )
    profiles = {}
    for line in list_result.stdout.splitlines():
        idx = line.find(":")
        if idx == -1:
            continue
        ctype = line[:idx].strip().lower()
        cname = line[idx+1:].strip()
        if ctype == "802-11-wireless" and cname:
            profiles[cname] = {"ssid": None, "priority": 0}

    for pname in list(profiles.keys()):
        detail = subprocess.run(
            ["nmcli", "--escape", "no", "-g",
             "802-11-wireless.ssid,connection.autoconnect-priority",
             "connection", "show", pname],
            capture_output=True, text=True, timeout=5
        )
        lines = detail.stdout.strip().splitlines()
        ssid     = lines[0].strip() if len(lines) > 0 else ""
        priority = lines[1].strip() if len(lines) > 1 else "0"
        profiles[pname]["ssid"]     = ssid if ssid else pname
        try:
            profiles[pname]["priority"] = int(priority)
        except ValueError:
            profiles[pname]["priority"] = 0

    return profiles  # {profile_name: {ssid, priority}}


def _nm_saved_wifi_ssids():
    """Return a set of SSIDs (not profile names) for all saved WiFi profiles."""
    return {v["ssid"] for v in _nm_wifi_profiles().values()}


def _nm_active_wifi():
    """Return (ssid, ip) of the currently active WiFi connection, or (None, None)."""
    import re as _re

    # ── Get IP directly from kernel via `ip -4 addr show wlan0` ──────────────
    # This is the most reliable source — unaffected by NM state or output format
    ip = None
    try:
        ip_out = subprocess.run(
            ["ip", "-4", "addr", "show", "wlan0"],
            capture_output=True, text=True, timeout=5
        )
        m = _re.search(r"inet (\d+\.\d+\.\d+\.\d+)", ip_out.stdout)
        if m:
            ip = m.group(1)
    except Exception:
        pass

    # ── Get active connection profile name from nmcli ─────────────────────────
    # nmcli -g outputs values only (no keys), one per line, in the order requested
    profile_name = None
    try:
        result = subprocess.run(
            ["nmcli", "--escape", "no", "-g",
             "GENERAL.CONNECTION", "dev", "show", "wlan0"],
            capture_output=True, text=True, timeout=5
        )
        val = result.stdout.strip()
        if val and val != "--":
            profile_name = val
    except Exception:
        pass

    if not profile_name:
        return None, ip

    # ── Resolve profile name → actual SSID ───────────────────────────────────
    profiles = _nm_wifi_profiles()
    if profile_name in profiles:
        ssid = profiles[profile_name]["ssid"]
    else:
        try:
            detail = subprocess.run(
                ["nmcli", "--escape", "no", "-g", "802-11-wireless.ssid",
                 "connection", "show", profile_name],
                capture_output=True, text=True, timeout=5
            )
            ssid = detail.stdout.strip() or profile_name
        except Exception:
            ssid = profile_name

    return ssid, ip


@app.route("/api/wifi/debug", methods=["GET"])
def api_wifi_debug():
    """Return raw nmcli and ip addr output for diagnosing IP detection issues."""
    try:
        nmcli_out = subprocess.run(
            ["nmcli", "--escape", "no", "dev", "show", "wlan0"],
            capture_output=True, text=True, timeout=5
        ).stdout
        ip_addr_out = subprocess.run(
            ["ip", "-4", "addr", "show", "wlan0"],
            capture_output=True, text=True, timeout=5
        ).stdout
        ssid, ip = _nm_active_wifi()
        return jsonify({
            "ok": True,
            "resolved_ssid": ssid,
            "resolved_ip": ip,
            "nmcli_raw": nmcli_out,
            "ip_addr_raw": ip_addr_out,
        })
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/api/wifi/status", methods=["GET"])
def api_wifi_status():
    """Return current WiFi SSID and IP."""
    try:
        ssid, ip = _nm_active_wifi()
        return jsonify({"ok": True, "ssid": ssid, "ip": ip})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/api/wifi/scan", methods=["GET"])
def api_wifi_scan():
    """Return visible SSIDs sorted by signal strength, flagging saved profiles."""
    try:
        subprocess.run(
            ["sudo", "nmcli", "dev", "wifi", "rescan"],
            capture_output=True, timeout=8
        )
        # Use --escape no so colons in SSIDs don't break parsing
        result = subprocess.run(
            ["nmcli", "--escape", "no", "-t", "-f", "SSID,SIGNAL", "dev", "wifi", "list"],
            capture_output=True, text=True, timeout=10
        )
        saved_ssids = _nm_saved_wifi_ssids()
        seen  = {}
        for line in result.stdout.splitlines():
            # Last field is signal (integer), everything before the last colon is SSID
            idx = line.rfind(":")
            if idx == -1:
                continue
            ssid = line[:idx].strip()
            if not ssid:
                continue
            try:
                signal = int(line[idx+1:].strip())
            except ValueError:
                signal = 0
            signal_dbm = -100 + signal
            if ssid not in seen or signal_dbm > seen[ssid]["signal"]:
                seen[ssid] = {
                    "ssid":   ssid,
                    "signal": signal_dbm,
                    "saved":  ssid in saved_ssids,
                }
        sorted_nets = sorted(seen.values(), key=lambda x: x["signal"], reverse=True)
        return jsonify({"ok": True, "networks": sorted_nets})
    except Exception as e:
        logger.error(f"WiFi scan error: {e}")
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/api/wifi/saved", methods=["GET"])
def api_wifi_saved():
    """Return all saved WiFi profiles with SSID, priority, and default flag."""
    try:
        profiles = _nm_wifi_profiles()
        entries  = [{"ssid": v["ssid"], "priority": v["priority"]}
                    for v in profiles.values()]
        entries.sort(key=lambda x: (-x["priority"], x["ssid"].lower()))
        max_pri  = max((e["priority"] for e in entries), default=0)
        for e in entries:
            e["default"] = (e["priority"] == max_pri and max_pri > 0)
        return jsonify({"ok": True, "saved": entries})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/api/wifi/connect", methods=["POST"])
def api_wifi_connect():
    """Connect to a WiFi network. Uses saved profile if no password supplied."""
    data     = request.json or {}
    ssid     = data.get("ssid", "").strip()
    password = data.get("password", "").strip()
    if not ssid:
        return jsonify({"ok": False, "error": "SSID required"}), 400
    try:
        profiles = _nm_wifi_profiles()  # {profile_name: {ssid, priority}}
        # Find the profile whose SSID matches (if any)
        profile_name = next((p for p, v in profiles.items() if v["ssid"] == ssid), None)

        # Disconnect current connection first so NM doesn't block the switch
        subprocess.run(
            ["sudo", "nmcli", "dev", "disconnect", "wlan0"],
            capture_output=True, timeout=8
        )

        if profile_name and not password:
            # Saved profile exists — bring it up by profile name
            cmd = ["sudo", "nmcli", "connection", "up", profile_name,
                   "ifname", "wlan0"]
        elif password:
            # Delete stale profile if one exists, then connect fresh with password
            if profile_name:
                subprocess.run(
                    ["sudo", "nmcli", "connection", "delete", profile_name],
                    capture_output=True, timeout=8
                )
            cmd = [
                "sudo", "nmcli", "dev", "wifi", "connect", ssid,
                "password", password,
                "ifname", "wlan0",
            ]
        else:
            # Open network, no saved profile
            cmd = [
                "sudo", "nmcli", "dev", "wifi", "connect", ssid,
                "ifname", "wlan0",
            ]

        subprocess.Popen(cmd)
        return jsonify({"ok": True, "message": f"Connecting to '{ssid}'…"})
    except Exception as e:
        logger.error(f"WiFi connect error: {e}")
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/api/wifi/set-priority", methods=["POST"])
def api_wifi_set_priority():
    """Set one SSID as the default (priority 10); demote all others to 0."""
    data = request.json or {}
    ssid = data.get("ssid", "").strip()
    if not ssid:
        return jsonify({"ok": False, "error": "SSID required"}), 400
    try:
        profiles = _nm_wifi_profiles()
        for pname, pdata in profiles.items():
            priority = 10 if pdata["ssid"] == ssid else 0
            subprocess.run(
                ["sudo", "nmcli", "connection", "modify", pname,
                 "connection.autoconnect-priority", str(priority)],
                capture_output=True, timeout=8
            )
        return jsonify({"ok": True, "message": f"'{ssid}' set as default network"})
    except Exception as e:
        logger.error(f"WiFi set-priority error: {e}")
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/api/wifi/forget", methods=["POST"])
def api_wifi_forget():
    """Delete the saved WiFi profile whose SSID matches the request."""
    data = request.json or {}
    ssid = data.get("ssid", "").strip()
    if not ssid:
        return jsonify({"ok": False, "error": "SSID required"}), 400
    try:
        profiles = _nm_wifi_profiles()
        profile_name = next((p for p, v in profiles.items() if v["ssid"] == ssid), None)
        if not profile_name:
            return jsonify({"ok": False, "error": f"No saved profile found for '{ssid}'"}), 404
        result = subprocess.run(
            ["sudo", "nmcli", "connection", "delete", profile_name],
            capture_output=True, text=True, timeout=8
        )
        if result.returncode == 0:
            return jsonify({"ok": True, "message": f"Forgot '{ssid}'"})
        else:
            err = result.stderr.strip() or result.stdout.strip()
            return jsonify({"ok": False, "error": err}), 500
    except Exception as e:
        logger.error(f"WiFi forget error: {e}")
        return jsonify({"ok": False, "error": str(e)}), 500



@app.route("/api/shutdown", methods=["POST"])
def api_shutdown():
    kiln.stop()
    close_session(db, _current_session_id)
    threading.Thread(target=lambda: (
        __import__('time').sleep(1),
        subprocess.call(['sudo', 'shutdown', 'now'])
    ), daemon=True).start()
    return jsonify({"ok": True, "message": "Shutting down..."})


# ── Serve UI ──────────────────────────────────────────────────────────────────

@app.route("/")
def index():
    return send_from_directory("static", "index.html")


@app.route("/<path:path>")
def static_files(path):
    return send_from_directory("static", path)


# ── Main ──────────────────────────────────────────────────────────────────────

def start_shutdown_button(pin=21):
    """Monitor GPIO pin for physical shutdown button (pin to GND)."""
    try:
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        logger.info(f"Shutdown button active on GPIO{pin}")
        def watch():
            import time
            while True:
                if GPIO.input(pin) == GPIO.LOW:
                    logger.info("Shutdown button pressed — shutting down")
                    kiln.stop()
                    time.sleep(0.5)
                    subprocess.call(['sudo', 'shutdown', 'now'])
                    break
                time.sleep(0.1)
        threading.Thread(target=watch, daemon=True).start()
    except Exception as e:
        logger.warning(f"GPIO shutdown button unavailable: {e}")


if __name__ == "__main__":
    kiln.start()
    start_shutdown_button()
    logger.info(f"Starting on {HOST}:{PORT}")
    app.run(host=HOST, port=PORT, debug=False, threaded=True)
