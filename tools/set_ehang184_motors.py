#!/usr/bin/env python3
"""
Set Ehang 184 motor control allocation parameters via MAVLink.

Motor layout (viewed from above, nose up):

          FRONT
       0       1
      (CW)   (CCW)
       FL      FR
        \      /
         \    /
         /    \
        /      \
       RL      RR
      (CCW)   (CW)
       2       3

PX4 body frame: X+ forward, Y+ right, Z+ down
KM sign: negative = CW, positive = CCW

Usage:
    python3 set_ehang184_motors.py [--host 127.0.0.1] [--port 14540]
"""

import time
import argparse
from pymavlink import mavutil

REAL32 = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
INT32  = mavutil.mavlink.MAV_PARAM_TYPE_INT32

MOTOR_PARAMS = [
    # Control allocator type
    ("CA_AIRFRAME",    0,     INT32),
    ("CA_ROTOR_COUNT", 4,     INT32),
    ("CA_METHOD",      2,     INT32),
    ("CA_FAILURE_MODE",0,     INT32),

    # Motor 0 — Front-Left, CW  (PY=-1.4 = left, KM=-0.05 = CW)
    ("CA_ROTOR0_PX",   1.4,   REAL32),
    ("CA_ROTOR0_PY",  -1.4,   REAL32),
    ("CA_ROTOR0_AZ",  -1.0,   REAL32),
    ("CA_ROTOR0_CT",   6.5,   REAL32),
    ("CA_ROTOR0_KM",  -0.05,  REAL32),

    # Motor 1 — Front-Right, CCW  (PY=+1.4 = right, KM=+0.05 = CCW)
    ("CA_ROTOR1_PX",   1.4,   REAL32),
    ("CA_ROTOR1_PY",   1.4,   REAL32),
    ("CA_ROTOR1_AZ",  -1.0,   REAL32),
    ("CA_ROTOR1_CT",   6.5,   REAL32),
    ("CA_ROTOR1_KM",   0.05,  REAL32),

    # Motor 2 — Rear-Left, CCW  (PY=-1.4 = left, KM=+0.05 = CCW)
    ("CA_ROTOR2_PX",  -1.4,   REAL32),
    ("CA_ROTOR2_PY",  -1.4,   REAL32),
    ("CA_ROTOR2_AZ",  -1.0,   REAL32),
    ("CA_ROTOR2_CT",   6.5,   REAL32),
    ("CA_ROTOR2_KM",   0.05,  REAL32),

    # Motor 3 — Rear-Right, CW  (PY=+1.4 = right, KM=-0.05 = CW)
    ("CA_ROTOR3_PX",  -1.4,   REAL32),
    ("CA_ROTOR3_PY",   1.4,   REAL32),
    ("CA_ROTOR3_AZ",  -1.0,   REAL32),
    ("CA_ROTOR3_CT",   6.5,   REAL32),
    ("CA_ROTOR3_KM",  -0.05,  REAL32),

    # PWM output → motor index mapping
    ("PWM_MAIN_FUNC1", 101,   INT32),   # Motor 0 (FL CW)
    ("PWM_MAIN_FUNC2", 102,   INT32),   # Motor 1 (FR CCW)
    ("PWM_MAIN_FUNC3", 103,   INT32),   # Motor 2 (RL CCW)
    ("PWM_MAIN_FUNC4", 104,   INT32),   # Motor 3 (RR CW)
]


def set_param(conn, name, value, ptype, retries=3, ack_timeout=3.0):
    for attempt in range(1, retries + 1):
        conn.mav.param_set_send(
            conn.target_system, conn.target_component,
            name.encode(), float(value), ptype)
        deadline = time.time() + ack_timeout
        while time.time() < deadline:
            msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg and msg.param_id.rstrip("\x00") == name:
                return msg.param_value
        if attempt < retries:
            print(f"  [retry {attempt}] {name}")
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=14540)
    args = ap.parse_args()

    conn = mavutil.mavlink_connection(
        f"udp:{args.host}:{args.port}", source_system=254)
    print("Waiting for heartbeat...")
    conn.wait_heartbeat(timeout=15)
    print(f"Connected — sysid={conn.target_system}\n")

    ok = fail = 0
    for name, value, ptype in MOTOR_PARAMS:
        acked = set_param(conn, name, value, ptype)
        if acked is not None:
            print(f"  {name:22s}  {value:>8.4f}  OK")
            ok += 1
        else:
            print(f"  {name:22s}  FAILED")
            fail += 1

    print(f"\nDone: {ok} set, {fail} failed.")
    conn.close()


if __name__ == "__main__":
    main()
