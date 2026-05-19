#!/usr/bin/env python3
"""
record_flight.py — Record MAVLink telemetry to CSV for flight analysis.

Supports PX4 and ArduCopter (auto-detected from heartbeat).
Saves flight.csv + metadata.json to a timestamped session directory.

Usage:
    python3 tools/record_flight.py                    # UDP 14540 (PX4 default)
    python3 tools/record_flight.py --port 14570       # ArduCopter
    python3 tools/record_flight.py --port 14550       # QGC passthrough
    python3 tools/record_flight.py --timeout 120      # record 2 minutes then stop
    python3 tools/record_flight.py --out logs/sessions
    python3 tools/record_flight.py --armed-only       # only log while armed
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    from pymavlink import mavutil
except ImportError:
    print("ERROR: pymavlink not installed. Run: pip install pymavlink")
    sys.exit(1)

# ── Mode tables ──────────────────────────────────────────────────────────────

ARDU_COPTER_MODES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
    5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT",
    13: "SPORT", 15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE",
}

PX4_MAIN_MODES = {
    1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO",
    5: "ACRO", 6: "OFFBOARD", 7: "STABILIZED",
}
PX4_AUTO_SUB = {
    1: "READY", 2: "TAKEOFF", 3: "LOITER", 4: "MISSION",
    5: "RTL", 6: "LAND", 8: "FOLLOW",
}

CSV_FIELDS = [
    "time_s", "armed", "mode",
    "roll_deg", "pitch_deg", "yaw_deg",
    "rollrate_rads", "pitchrate_rads", "yawrate_rads",
    "alt_rel_m", "alt_msl_m",
    "vx_ms", "vy_ms", "vz_ms", "speed_ms",
    "throttle_pct",
    "lat_deg", "lon_deg",
    "gps_fix", "gps_sats",
    "batt_v", "batt_pct",
    "ekf_ok", "ekf_vvar", "ekf_hvar", "ekf_vvar_vert",
    "wp_dist_m",
]


def decode_mode(autopilot: int, mav_type: int, custom_mode: int) -> str:
    if autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
        return ARDU_COPTER_MODES.get(custom_mode, f"MODE{custom_mode}")
    main = (custom_mode >> 16) & 0xFF
    sub  = (custom_mode >> 24) & 0xFF
    name = PX4_MAIN_MODES.get(main, f"M{main}")
    if main == 4:
        name += "/" + PX4_AUTO_SUB.get(sub, str(sub))
    return name


def autopilot_name(autopilot: int) -> str:
    return {
        mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA: "ArduCopter",
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32: "PX4",  # alias
        12: "PX4",
    }.get(autopilot, f"AP{autopilot}")


# ── Shared telemetry state ────────────────────────────────────────────────────

class State:
    def __init__(self):
        self.autopilot = 0
        self.mav_type  = 0
        self.armed     = False
        self.mode      = "UNKNOWN"
        self.roll = self.pitch = self.yaw = 0.0
        self.rollrate = self.pitchrate = self.yawrate = 0.0
        self.alt_rel   = float("nan")
        self.alt_msl   = float("nan")
        self.vx = self.vy = self.vz = float("nan")
        self.speed     = float("nan")
        self.throttle  = float("nan")
        self.lat = self.lon = float("nan")
        self.gps_fix   = 0
        self.gps_sats  = 0
        self.batt_v    = float("nan")
        self.batt_pct  = float("nan")
        self.ekf_ok    = 1
        self.ekf_vvar  = float("nan")
        self.ekf_hvar  = float("nan")
        self.ekf_vvar_vert = float("nan")
        self.wp_dist   = float("nan")

    def to_row(self, t: float) -> dict:
        return {
            "time_s":       f"{t:.3f}",
            "armed":        int(self.armed),
            "mode":         self.mode,
            "roll_deg":     _fmt(math.degrees(self.roll)),
            "pitch_deg":    _fmt(math.degrees(self.pitch)),
            "yaw_deg":      _fmt(math.degrees(self.yaw)),
            "rollrate_rads":  _fmt(self.rollrate),
            "pitchrate_rads": _fmt(self.pitchrate),
            "yawrate_rads":   _fmt(self.yawrate),
            "alt_rel_m":    _fmt(self.alt_rel),
            "alt_msl_m":    _fmt(self.alt_msl),
            "vx_ms":        _fmt(self.vx),
            "vy_ms":        _fmt(self.vy),
            "vz_ms":        _fmt(self.vz),
            "speed_ms":     _fmt(self.speed),
            "throttle_pct": _fmt(self.throttle),
            "lat_deg":      _fmt(self.lat, 7),
            "lon_deg":      _fmt(self.lon, 7),
            "gps_fix":      self.gps_fix,
            "gps_sats":     self.gps_sats,
            "batt_v":       _fmt(self.batt_v),
            "batt_pct":     _fmt(self.batt_pct),
            "ekf_ok":       self.ekf_ok,
            "ekf_vvar":     _fmt(self.ekf_vvar),
            "ekf_hvar":     _fmt(self.ekf_hvar),
            "ekf_vvar_vert":_fmt(self.ekf_vvar_vert),
            "wp_dist_m":    _fmt(self.wp_dist),
        }


def _fmt(v: float, decimals: int = 4) -> str:
    if not math.isfinite(v):
        return ""
    return f"{v:.{decimals}f}"


# ── MAVLink message handlers ─────────────────────────────────────────────────

def handle_heartbeat(msg, state: State):
    state.autopilot = msg.autopilot
    state.mav_type  = msg.type
    state.armed     = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    state.mode      = decode_mode(msg.autopilot, msg.type, msg.custom_mode)


def handle_attitude(msg, state: State):
    state.roll      = msg.roll
    state.pitch     = msg.pitch
    state.yaw       = msg.yaw
    state.rollrate  = msg.rollspeed
    state.pitchrate = msg.pitchspeed
    state.yawrate   = msg.yawspeed


def handle_global_pos(msg, state: State):
    state.lat     = msg.lat * 1e-7
    state.lon     = msg.lon * 1e-7
    state.alt_msl = msg.alt * 1e-3
    state.alt_rel = msg.relative_alt * 1e-3
    state.vx      = msg.vx * 1e-2
    state.vy      = msg.vy * 1e-2
    state.vz      = msg.vz * 1e-2
    state.speed   = math.hypot(state.vx, state.vy)


def handle_vfr_hud(msg, state: State):
    state.throttle = msg.throttle
    if not math.isfinite(state.speed):
        state.speed = msg.groundspeed


def handle_gps_raw(msg, state: State):
    state.gps_fix  = msg.fix_type
    state.gps_sats = msg.satellites_visible


def handle_battery(msg, state: State):
    if msg.voltages and msg.voltages[0] != 65535:
        state.batt_v = msg.voltages[0] * 1e-3
    if msg.battery_remaining >= 0:
        state.batt_pct = msg.battery_remaining


def handle_ekf_status(msg, state: State):
    state.ekf_ok       = int(bool(msg.flags & 0x1F))
    state.ekf_vvar     = msg.velocity_variance
    state.ekf_hvar     = msg.pos_horiz_variance
    state.ekf_vvar_vert = msg.pos_vert_variance


def handle_nav_output(msg, state: State):
    state.wp_dist = msg.wp_dist


HANDLERS = {
    "HEARTBEAT":          handle_heartbeat,
    "ATTITUDE":           handle_attitude,
    "GLOBAL_POSITION_INT":handle_global_pos,
    "VFR_HUD":            handle_vfr_hud,
    "GPS_RAW_INT":        handle_gps_raw,
    "BATTERY_STATUS":     handle_battery,
    "EKF_STATUS_REPORT":  handle_ekf_status,
    "NAV_CONTROLLER_OUTPUT": handle_nav_output,
}


# ── Connection ───────────────────────────────────────────────────────────────

def connect(port: int, host: str) -> mavutil.mavudp:
    print(f"Connecting to {host}:{port} ...")
    conn = mavutil.mavlink_connection(f"udp:{host}:{port}", source_system=254)
    deadline = time.time() + 30
    while time.time() < deadline:
        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if msg and msg.get_srcSystem() not in (0, 255):
            conn.target_system    = msg.get_srcSystem()
            conn.target_component = msg.get_srcComponent()
            ap = autopilot_name(msg.autopilot)
            print(f"  Connected  sysid={conn.target_system}  autopilot={ap}\n")
            return conn
    print("ERROR: no heartbeat within 30 s", file=sys.stderr)
    sys.exit(1)


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host",        default="127.0.0.1")
    ap.add_argument("--port",        type=int, default=14540)
    ap.add_argument("--timeout",     type=float, default=0,
                    help="Stop after N seconds (0 = run until Ctrl-C)")
    ap.add_argument("--out",         default="logs/sessions",
                    help="Parent directory for session folders")
    ap.add_argument("--rate",        type=float, default=4.0,
                    help="Approx. rows per second to record (default 4)")
    ap.add_argument("--armed-only",  action="store_true",
                    help="Only write rows while vehicle is armed")
    args = ap.parse_args()

    conn  = connect(args.port, args.host)
    state = State()

    # First heartbeat to capture autopilot type
    for _ in range(5):
        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if msg and msg.get_srcSystem() == conn.target_system:
            handle_heartbeat(msg, state)
            break

    # Session directory
    stamp   = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    out_dir = Path(args.out) / stamp
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path  = out_dir / "flight.csv"
    meta_path = out_dir / "metadata.json"

    metadata = {
        "session":   stamp,
        "port":      args.port,
        "autopilot": autopilot_name(state.autopilot),
        "started":   datetime.now().isoformat(),
    }

    print(f"Recording to {csv_path}")
    print("Press Ctrl-C to stop.\n")

    stop = False
    def _sig(*_): nonlocal stop; stop = True
    signal.signal(signal.SIGINT,  _sig)
    signal.signal(signal.SIGTERM, _sig)

    interval  = 1.0 / max(args.rate, 0.5)
    t0        = time.time()
    last_row  = t0 - interval
    row_count = 0

    with open(csv_path, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=CSV_FIELDS)
        writer.writeheader()

        while not stop:
            if args.timeout > 0 and (time.time() - t0) >= args.timeout:
                break

            msg = conn.recv_match(blocking=True, timeout=0.1)
            if msg:
                mtype = msg.get_type()
                if mtype in HANDLERS:
                    HANDLERS[mtype](msg, state)

            now = time.time()
            if (now - last_row) >= interval:
                last_row = now
                if args.armed_only and not state.armed:
                    continue
                writer.writerow(state.to_row(now - t0))
                row_count += 1

                armed_str = "ARMED" if state.armed else "disarmed"
                alt_str   = f"{state.alt_rel:.1f}m" if math.isfinite(state.alt_rel) else "---"
                roll_str  = f"{math.degrees(state.roll):+.1f}°"
                pitch_str = f"{math.degrees(state.pitch):+.1f}°"
                print(f"\r  {armed_str:<10} {state.mode:<16} "
                      f"alt={alt_str:<8} roll={roll_str:<8} pitch={pitch_str:<8} "
                      f"rows={row_count}", end="", flush=True)

    print()
    metadata["rows"]    = row_count
    metadata["duration_s"] = round(time.time() - t0, 1)
    meta_path.write_text(json.dumps(metadata, indent=2))
    print(f"\nSaved {row_count} rows → {out_dir}")
    conn.close()


if __name__ == "__main__":
    main()
