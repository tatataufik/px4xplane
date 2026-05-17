#!/usr/bin/env python3
"""
Arm or disarm PX4 via RC stick gesture using RC_CHANNELS_OVERRIDE.

RC_CHANNELS_OVERRIDE must be sent at >=10Hz continuously for the full
COM_RC_ARM_HYST duration (default 1000ms). Slower rates cause rc_update
to drop packets and the hysteresis never completes.

Channel mapping (Mode 2):
  CH1=Roll  CH2=Pitch  CH3=Throttle  CH4=Yaw  CH5=FlightMode

Arm gesture:   CH3=1000, CH4=2000, CH1=1500, CH2=1500, CH5=1000 (MANUAL)
Disarm gesture: CH3=1000, CH4=1000, CH1=1500, CH2=1500, CH5=1000 (MANUAL)

Usage:
  python3 gesture_arm.py arm   [--host 127.0.0.1] [--port 14540]
  python3 gesture_arm.py disarm [--host 127.0.0.1] [--port 14540]
"""
import sys
import time
import argparse
from pymavlink import mavutil

RATE_HZ = 10          # send rate — must be > 3Hz, 10Hz recommended
HOLD_S  = 3.5         # hold duration — must be > COM_RC_ARM_HYST (3.0s)
SWITCH_S = 0.5        # time to send CH5=MANUAL before gesture

def send_rc(conn, ch1=1500, ch2=1500, ch3=1000, ch4=1500, ch5=1000):
    conn.mav.rc_channels_override_send(
        conn.target_system, conn.target_component,
        ch1, ch2, ch3, ch4, ch5, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

def is_armed(conn):
    hb = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) if hb else None

def send_loop(conn, ch4_value, label):
    interval = 1.0 / RATE_HZ

    # Phase 1: switch to MANUAL mode on CH5
    print(f"Switching to MANUAL mode (CH5=1000) for {SWITCH_S}s...")
    t0 = time.time()
    while time.time() - t0 < SWITCH_S:
        send_rc(conn, ch5=1000)
        time.sleep(interval)

    # Phase 2: hold gesture
    print(f"Holding {label} gesture at {RATE_HZ}Hz for {HOLD_S}s...")
    t0 = time.time()
    count = 0
    while time.time() - t0 < HOLD_S:
        send_rc(conn, ch1=1500, ch2=1500, ch3=1000, ch4=ch4_value, ch5=1000)
        count += 1
        time.sleep(interval)
    print(f"  Sent {count} packets over {HOLD_S}s")

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('action', choices=['arm', 'disarm'])
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=14540)
    args = ap.parse_args()

    conn = mavutil.mavlink_connection(
        f'udp:{args.host}:{args.port}', source_system=255)
    print("Waiting for heartbeat...")
    conn.wait_heartbeat()

    armed = is_armed(conn)
    print(f"Connected — sysid={conn.target_system}  mode={conn.flightmode}  armed={armed}")

    if args.action == 'arm':
        if armed:
            print("Already armed.")
            sys.exit(0)
        send_loop(conn, ch4_value=2000, label='ARM  (CH4=2000)')
    else:
        if not armed:
            print("Already disarmed.")
            sys.exit(0)
        send_loop(conn, ch4_value=1000, label='DISARM (CH4=1000)')

    time.sleep(0.3)
    result = is_armed(conn)
    print(f"\nResult: armed={result}  ({'OK' if (result == (args.action == 'arm')) else 'FAILED'})")

    # drain any status messages
    deadline = time.time() + 1.0
    while time.time() < deadline:
        msg = conn.recv_match(type='STATUSTEXT', blocking=False)
        if msg:
            print(f"  PX4: [{msg.severity}] {msg.text.rstrip()}")
        time.sleep(0.02)

if __name__ == '__main__':
    main()
