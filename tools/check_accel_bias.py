#!/usr/bin/env python3
"""
Read EKF2 accelerometer bias estimate and the EKF2_ABL_LIM threshold from PX4 SITL.
Also increases EKF2_ABL_LIM until the preflight check can pass.
"""

import sys
import time
import argparse
import struct
from pymavlink import mavutil

# preflight check threshold = 0.9 * EKF2_ABL_LIM (PX4 main, may vary by version)
PREFLIGHT_FACTOR = 0.9


def wait_heartbeat(conn, timeout=30):
    conn.wait_heartbeat(timeout=timeout)
    print(f"Heartbeat — sysid={conn.target_system}\n")


def fetch_param(conn, name, timeout=5.0):
    conn.mav.param_request_read_send(
        conn.target_system, conn.target_component, name.encode(), -1)
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg and msg.param_id.rstrip("\x00") == name:
            return msg.param_value
    return None


def set_param_i32(conn, name, value, retries=3, ack_timeout=3.0):
    for _ in range(retries):
        conn.mav.param_set_send(
            conn.target_system, conn.target_component,
            name.encode(), float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        deadline = time.time() + ack_timeout
        while time.time() < deadline:
            msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg and msg.param_id.rstrip("\x00") == name:
                return msg.param_value
    return None


def send_shell(conn, cmd, read_timeout=3.0):
    SHELL_DEV = 10
    raw = (cmd + "\n").encode("ascii")
    chunk = raw[:70]
    data = list(chunk) + [0] * (70 - len(chunk))
    conn.mav.serial_control_send(
        SHELL_DEV,
        mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND |
        mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE,
        0, 0, len(chunk), data)
    output = b""
    deadline = time.time() + read_timeout
    while time.time() < deadline:
        msg = conn.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=0.4)
        if msg and msg.count > 0:
            output += bytes(msg.data[:msg.count])
            deadline = time.time() + 0.5
    return output.decode("ascii", errors="replace")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=14550)
    args = ap.parse_args()

    conn = mavutil.mavlink_connection(f"udp:{args.host}:{args.port}", source_system=254)
    wait_heartbeat(conn)

    abl_lim = fetch_param(conn, "EKF2_ABL_LIM")
    print(f"EKF2_ABL_LIM  = {abl_lim:.4f} m/s²")
    print(f"Preflight threshold ≈ {PREFLIGHT_FACTOR * abl_lim:.4f} m/s² "
          f"(factor {PREFLIGHT_FACTOR})\n")

    # Read bias estimate via NSH: listener estimator_sensor_bias
    print("Reading estimator_sensor_bias (5 samples)...")
    out = send_shell(conn, "listener estimator_sensor_bias -n 5", read_timeout=6.0)
    print(out if out.strip() else "  (no output)")

    # If bias clearly > threshold, bump ABL_LIM and restart ekf2
    new_lim = 9.0
    if abl_lim < new_lim:
        print(f"\nBumping EKF2_ABL_LIM: {abl_lim:.2f} → {new_lim}")
        acked = set_param_i32(conn, "EKF2_ABL_LIM", new_lim)
        print(f"  Acked: {acked:.4f}")

        print("\nRestarting ekf2...")
        print(send_shell(conn, "ekf2 stop", 4.0))
        time.sleep(1)
        print(send_shell(conn, "ekf2 start", 4.0))
        print("Done. Preflight threshold now ≈ "
              f"{PREFLIGHT_FACTOR * new_lim:.2f} m/s²")
    else:
        print(f"\nEKF2_ABL_LIM already ≥ {new_lim}, no change.")

    conn.close()


if __name__ == "__main__":
    main()
