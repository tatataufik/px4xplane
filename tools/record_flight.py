#!/usr/bin/env python3
"""
record_flight.py — Capture PX4 MAVLink telemetry to CSV for offline analysis.

Connects to PX4 via pymavlink, records key messages (HEARTBEAT, ATTITUDE,
NAV_CONTROLLER_OUTPUT, MISSION_CURRENT, VFR_HUD, GLOBAL_POSITION_INT) as a
time-aligned flat CSV, one row per ATTITUDE message receipt (~10-50 Hz).

Connection options (--connection argument):
  TELEM1 serial (preferred, no PPP overhead):
    serial:/dev/ttyUSB0:921600
    serial:/dev/ttyUSB1:921600

  UDP broadcast from PX4 GCS port (rc.board_ppp adds mavlink -u 14556 -p):
    udpin:0.0.0.0:14556

  Standard SITL UDP (PX4 SITL on same machine):
    udpout:127.0.0.1:14550
    udpin:0.0.0.0:14550

Usage:
    python3 tools/record_flight.py
    python3 tools/record_flight.py --connection serial:/dev/ttyUSB0:921600
    python3 tools/record_flight.py --connection udpin:0.0.0.0:14556 --out /tmp

Requirements:
    pip install pymavlink
"""

import argparse
import csv
import os
import signal
import sys
import time
from datetime import datetime

try:
    from pymavlink import mavutil
except ImportError:
    print("ERROR: pymavlink not installed.  Run: pip install pymavlink")
    sys.exit(1)


# ── PX4 custom_mode decoding ──────────────────────────────────────────────────
# union px4_custom_mode { uint16_t reserved; uint8_t main_mode; uint8_t sub_mode; }
# In the 32-bit value (little-endian): main_mode = bits 16-23, sub_mode = bits 24-31
MAIN_MODE_NAMES = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO",
                   5: "ACRO", 6: "OFFBOARD", 7: "STABILIZED", 8: "RATTITUDE"}
SUB_MODE_AUTO_NAMES = {1: "READY", 2: "TAKEOFF", 3: "LOITER", 4: "MISSION",
                       5: "RTL", 6: "LAND", 8: "FOLLOW_TARGET"}


def decode_custom_mode(custom_mode):
    main = (custom_mode >> 16) & 0xFF
    sub  = (custom_mode >> 24) & 0xFF
    return main, sub


def is_armed(base_mode):
    return bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


# ── shared state (updated by message handlers, snapshotted on ATTITUDE) ───────
state = {
    # HEARTBEAT
    "base_mode": 0, "custom_mode": 0, "main_mode": 0, "sub_mode": 0, "armed": False,
    # ATTITUDE
    "boot_ms": 0,
    "roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0,
    "rollrate_rads": 0.0, "pitchrate_rads": 0.0, "yawrate_rads": 0.0,
    # VFR_HUD
    "airspeed_ms": 0.0, "groundspeed_ms": 0.0, "heading_deg": 0,
    "throttle_pct": 0, "climb_ms": 0.0,
    # GLOBAL_POSITION_INT
    "lat_deg": 0.0, "lon_deg": 0.0, "alt_m": 0.0, "rel_alt_m": 0.0,
    # NAV_CONTROLLER_OUTPUT
    "nav_bearing_deg": 0, "target_bearing_deg": 0,
    "xtrack_error_m": 0.0, "wp_dist_m": 0,
    "nav_roll_deg": 0.0, "nav_pitch_deg": 0.0,
    # MISSION_CURRENT
    "mission_seq": 0, "mission_mode_flag": 0,
}

# ── message handlers ──────────────────────────────────────────────────────────
def handle_heartbeat(msg):
    state["base_mode"]   = msg.base_mode
    state["custom_mode"] = msg.custom_mode
    m, s = decode_custom_mode(msg.custom_mode)
    state["main_mode"] = m
    state["sub_mode"]  = s
    state["armed"]     = is_armed(msg.base_mode)


def handle_attitude(msg):
    import math
    state["boot_ms"]       = msg.time_boot_ms
    state["roll_deg"]      = math.degrees(msg.roll)
    state["pitch_deg"]     = math.degrees(msg.pitch)
    state["yaw_deg"]       = math.degrees(msg.yaw)
    state["rollrate_rads"] = msg.rollspeed
    state["pitchrate_rads"]= msg.pitchspeed
    state["yawrate_rads"]  = msg.yawspeed


def handle_vfr_hud(msg):
    state["airspeed_ms"]   = msg.airspeed
    state["groundspeed_ms"]= msg.groundspeed
    state["heading_deg"]   = msg.heading
    state["throttle_pct"]  = msg.throttle
    state["climb_ms"]      = msg.climb


def handle_global_position(msg):
    state["lat_deg"]   = msg.lat / 1e7
    state["lon_deg"]   = msg.lon / 1e7
    state["alt_m"]     = msg.alt / 1000.0
    state["rel_alt_m"] = msg.relative_alt / 1000.0


def handle_nav_controller(msg):
    state["nav_bearing_deg"]    = msg.nav_bearing
    state["target_bearing_deg"] = msg.target_bearing
    state["xtrack_error_m"]     = msg.xtrack_error
    state["wp_dist_m"]          = msg.wp_dist
    state["nav_roll_deg"]       = msg.nav_roll
    state["nav_pitch_deg"]      = msg.nav_pitch


def handle_mission_current(msg):
    state["mission_seq"]       = msg.seq
    state["mission_mode_flag"] = getattr(msg, "mission_mode", 0)


HANDLERS = {
    "HEARTBEAT":            handle_heartbeat,
    "ATTITUDE":             handle_attitude,
    "VFR_HUD":              handle_vfr_hud,
    "GLOBAL_POSITION_INT":  handle_global_position,
    "NAV_CONTROLLER_OUTPUT":handle_nav_controller,
    "MISSION_CURRENT":      handle_mission_current,
}

CSV_FIELDS = [
    "wall_time_s", "boot_ms",
    "armed", "base_mode", "main_mode", "sub_mode", "mission_seq", "mission_mode_flag",
    "roll_deg", "pitch_deg", "yaw_deg", "rollrate_rads", "pitchrate_rads", "yawrate_rads",
    "heading_deg", "airspeed_ms", "groundspeed_ms", "climb_ms", "throttle_pct",
    "lat_deg", "lon_deg", "alt_m", "rel_alt_m",
    "nav_bearing_deg", "target_bearing_deg", "xtrack_error_m", "wp_dist_m",
    "nav_roll_deg", "nav_pitch_deg",
]


def request_streams(mav):
    """Ask PX4 to send the messages we care about."""
    t = mav.target_system
    c = mav.target_component

    # REQUEST_DATA_STREAM (deprecated but still works in PX4)
    streams = [
        (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10),  # ATTITUDE 10 Hz
        (mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 10),  # VFR_HUD 10 Hz
        (mavutil.mavlink.MAV_DATA_STREAM_POSITION, 5), # GPS  5 Hz
    ]
    for stream_id, rate in streams:
        mav.mav.request_data_stream_send(t, c, stream_id, rate, 1)
        time.sleep(0.05)

    # SET_MESSAGE_INTERVAL for messages not in standard streams
    def set_interval(msg_id, interval_us):
        mav.mav.command_long_send(t, c,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            float(msg_id), float(interval_us), 0, 0, 0, 0, 0)

    set_interval(62, 200000)   # NAV_CONTROLLER_OUTPUT  5 Hz
    set_interval(42, 500000)   # MISSION_CURRENT        2 Hz


def main():
    parser = argparse.ArgumentParser(description="Record PX4 MAVLink telemetry to CSV")
    parser.add_argument("--connection", default="serial:/dev/ttyUSB0:921600",
                        help="MAVLink connection string (default: serial:/dev/ttyUSB0:921600)")
    parser.add_argument("--out", default=".",
                        help="Output directory for CSV file (default: current dir)")
    parser.add_argument("--timeout", type=float, default=0,
                        help="Recording duration in seconds (0 = run until Ctrl+C)")
    parser.add_argument("--dialect", default="common",
                        help="MAVLink dialect (default: common)")
    args = parser.parse_args()

    # File path
    os.makedirs(args.out, exist_ok=True)
    ts  = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(args.out, f"flightlog_{ts}.csv")

    print(f"Connecting to: {args.connection}")
    print(f"Output CSV:    {csv_path}")
    print("Press Ctrl+C to stop recording.")

    # Connect
    try:
        mav = mavutil.mavlink_connection(args.connection, dialect=args.dialect)
    except Exception as e:
        print(f"ERROR connecting: {e}")
        sys.exit(1)

    # Wait for first heartbeat
    print("Waiting for heartbeat...", end="", flush=True)
    hb = mav.wait_heartbeat(timeout=30)
    if hb is None:
        print("\nERROR: no heartbeat received in 30 s")
        sys.exit(1)
    print(f" system {mav.target_system} component {mav.target_component}")

    # Request telemetry
    request_streams(mav)

    # Graceful Ctrl+C handling
    stop = {"flag": False}
    def on_sigint(sig, frame):
        stop["flag"] = True
    signal.signal(signal.SIGINT, on_sigint)

    row_count   = 0
    start_wall  = time.time()
    attitude_count = 0
    last_status = start_wall

    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()

        while not stop["flag"]:
            if args.timeout > 0 and (time.time() - start_wall) >= args.timeout:
                break

            msg = mav.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue

            mtype = msg.get_type()
            handler = HANDLERS.get(mtype)
            if handler:
                handler(msg)

            # Snapshot one CSV row per ATTITUDE receipt
            if mtype == "ATTITUDE":
                attitude_count += 1
                row = {"wall_time_s": f"{time.time() - start_wall:.4f}"}
                for k in CSV_FIELDS[1:]:
                    v = state.get(k, 0)
                    row[k] = 1 if v is True else (0 if v is False else v)
                writer.writerow(row)
                row_count += 1

                # Status every 5 s
                now = time.time()
                if now - last_status >= 5.0:
                    m_name = MAIN_MODE_NAMES.get(state["main_mode"], str(state["main_mode"]))
                    s_name = SUB_MODE_AUTO_NAMES.get(state["sub_mode"], str(state["sub_mode"])) \
                             if state["main_mode"] == 4 else ""
                    mode_str = f"{m_name}/{s_name}" if s_name else m_name
                    armed_str = "ARMED" if state["armed"] else "disarmed"
                    print(f"  +{now - start_wall:.0f}s | {row_count} rows | "
                          f"{armed_str} | mode={mode_str} | "
                          f"hdg={state['heading_deg']:.0f}° "
                          f"yawrate={state['yawrate_rads']:.3f} rad/s "
                          f"wp={state['mission_seq']}")
                    last_status = now

    elapsed = time.time() - start_wall
    avg_hz  = attitude_count / elapsed if elapsed > 0 else 0
    print(f"\nStopped. Recorded {row_count} rows in {elapsed:.1f} s "
          f"({avg_hz:.1f} Hz avg).")
    print(f"CSV saved: {csv_path}")
    print(f"\nAnalyze with:")
    print(f"  python3 tools/analyze_flight_turning.py {csv_path}")


if __name__ == "__main__":
    main()
