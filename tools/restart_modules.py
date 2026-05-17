#!/usr/bin/env python3
"""
Send shell commands to PX4 SITL via MAVLink SERIAL_CONTROL (NSH shell).
Restarts airspeed_selector and ekf2 modules without rebooting PX4.

Usage:
    python3 restart_modules.py [--host 127.0.0.1] [--port 14550]
"""

import sys
import time
import argparse
from pymavlink import mavutil

SHELL_DEV = 10   # SERIAL_CONTROL_DEV_SHELL
READ_TIMEOUT = 3.0

COMMANDS = [
    "airspeed_selector stop",
    "airspeed_selector start",
    # NOTE: do NOT stop/start ekf2 — it takes 10–30s to re-converge after restart
    # and triggers "ekf2 missing data" during that window. Parameters set via
    # PARAM_SET are picked up immediately without an ekf2 restart.
]


def wait_heartbeat(conn, timeout=30):
    print(f"Waiting for heartbeat (up to {timeout}s)...")
    conn.wait_heartbeat(timeout=timeout)
    print(f"  Got heartbeat — sysid={conn.target_system} compid={conn.target_component}\n")


def send_shell(conn, cmd, read_timeout=READ_TIMEOUT):
    """Send a command to the PX4 NSH shell and print the response."""
    print(f">> {cmd}")
    raw = (cmd + "\n").encode("ascii")
    # SERIAL_CONTROL data field is 70 bytes
    chunk = raw[:70]
    data = list(chunk) + [0] * (70 - len(chunk))
    conn.mav.serial_control_send(
        SHELL_DEV,
        mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND |
        mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE,
        0,    # timeout (ms, 0 = no serial timeout)
        0,    # baudrate (ignored for shell)
        len(chunk),
        data,
    )
    # Collect response lines
    deadline = time.time() + read_timeout
    output = b""
    while time.time() < deadline:
        msg = conn.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=0.4)
        if msg and msg.count > 0:
            output += bytes(msg.data[: msg.count])
            deadline = time.time() + 0.5  # extend on activity
    if output:
        text = output.decode("ascii", errors="replace").strip()
        for line in text.splitlines():
            print(f"   {line}")
    print()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=14550)
    args = ap.parse_args()

    conn_str = f"udp:{args.host}:{args.port}"
    print(f"Connecting to PX4 SITL at {conn_str}\n")
    conn = mavutil.mavlink_connection(conn_str, source_system=254)
    wait_heartbeat(conn)

    for cmd in COMMANDS:
        send_shell(conn, cmd)
        time.sleep(0.5)

    print("Done. Watch PX4 console — preflight warnings should clear within a few seconds.")
    conn.close()


if __name__ == "__main__":
    main()
