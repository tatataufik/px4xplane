#!/usr/bin/env python3
"""
Connect to PX4 SITL via MAVLink UDP 14550 and set EKF2 / calibration parameters.
Run while PX4 SITL + X-Plane are running.

Usage:
    python3 set_ekf2_params.py [--host 127.0.0.1] [--port 14550]
"""

import sys
import time
import argparse
from pymavlink import mavutil

# (name, value, mavlink_type)
# INT32 must be used for integer/enum params — PX4 rejects REAL32 for those.
PARAMS = [
    # --- EKF2 accelerometer bias ---
    ("EKF2_ABIAS_INIT",  0.3,   mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    ("EKF2_ABL_LIM",     6.0,   mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    ("EKF2_ABL_TAU",     0.8,   mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    ("EKF2_ABL_ACCLIM",  35.0,  mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    ("EKF2_ACC_NOISE",   1.5,   mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    ("EKF2_ACC_B_NOISE", 0.003, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),

    # --- EKF2 gyroscope ---
    ("EKF2_GYR_NOISE",   0.02,  mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    ("EKF2_GYR_B_NOISE", 0.001, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),

    # --- EKF2 barometer ---
    ("EKF2_BARO_NOISE",  0.02,  mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    ("EKF2_BARO_DELAY",  0.0,   mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    ("EKF2_BARO_GATE",   15.0,  mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    ("EKF2_BARO_CTRL",   1,     mavutil.mavlink.MAV_PARAM_TYPE_INT32),

    # --- EKF2 GPS ---
    ("EKF2_GPS_DELAY",   0.0,   mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    ("EKF2_GPS_P_NOISE", 0.2,   mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    ("EKF2_GPS_V_NOISE", 0.3,   mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    ("EKF2_GPS_P_GATE",  5.0,   mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    ("EKF2_GPS_V_GATE",  7.0,   mavutil.mavlink.MAV_PARAM_TYPE_REAL32),

    # --- EKF2 magnetometer ---
    ("EKF2_MAG_TYPE",    0,     mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("EKF2_HEAD_NOISE",  0.3,   mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    ("EKF2_MAG_NOISE",   0.05,  mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    ("EKF2_MAG_GATE",    5.0,   mavutil.mavlink.MAV_PARAM_TYPE_REAL32),

    # --- EKF2 airspeed / aux ---
    ("EKF2_ASP_DELAY",   0.0,   mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    ("EKF2_TAS_GATE",    5.0,   mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    ("EKF2_AGP_CTRL",    3,     mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("EKF2_AVEL_DELAY",  0.0,   mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    ("EKF2_WIND_NSD",    0.01,  mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    ("EKF2_REQ_VDRIFT",  1.0,   mavutil.mavlink.MAV_PARAM_TYPE_REAL32),

    # --- Baro calibration priority (prefer single baro) ---
    ("CAL_BARO0_PRIO",   100,   mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("CAL_BARO1_PRIO",   0,     mavutil.mavlink.MAV_PARAM_TYPE_INT32),
]


def wait_heartbeat(conn, timeout=30):
    print(f"Waiting for heartbeat from PX4 (up to {timeout}s)...")
    conn.wait_heartbeat(timeout=timeout)
    print(f"  Got heartbeat — sysid={conn.target_system} compid={conn.target_component}")


def fetch_param(conn, name, timeout=5.0):
    conn.mav.param_request_read_send(
        conn.target_system, conn.target_component,
        name.encode(), -1
    )
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg and msg.param_id.rstrip("\x00") == name:
            return msg.param_value
    return None


def set_param(conn, name, value, param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
              retries=3, ack_timeout=3.0):
    for attempt in range(1, retries + 1):
        conn.mav.param_set_send(
            conn.target_system, conn.target_component,
            name.encode(), float(value), param_type
        )
        deadline = time.time() + ack_timeout
        while time.time() < deadline:
            msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg and msg.param_id.rstrip("\x00") == name:
                return msg.param_value
        print(f"  [attempt {attempt}] no ack for {name}, retrying...")
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=14550)
    args = ap.parse_args()

    conn_str = f"udp:{args.host}:{args.port}"
    print(f"Connecting to PX4 SITL at {conn_str}")
    conn = mavutil.mavlink_connection(conn_str, source_system=254)

    wait_heartbeat(conn)

    print(f"\nSetting {len(PARAMS)} parameters:\n")
    ok = 0
    fail = 0
    for name, target, ptype in PARAMS:
        current = fetch_param(conn, name)
        acked = set_param(conn, name, target, ptype)
        if acked is not None:
            cur_str = f"{current:.4f} ->" if current is not None else ""
            print(f"  {name:30s}  {cur_str:>14}  {acked:.4f}  OK")
            ok += 1
        else:
            print(f"  {name:30s}  FAILED (no ack)")
            fail += 1

    print(f"\nDone: {ok} set, {fail} failed.")
    if fail == 0:
        print("\nAll parameters applied. EKF2 will use them immediately.")
        print("Note: reboot_required params (e.g. EKF2_HGT_REF) need a PX4 restart.")
    conn.close()


if __name__ == "__main__":
    main()
