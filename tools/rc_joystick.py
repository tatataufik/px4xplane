#!/usr/bin/env python3
"""
rc_joystick.py — Joystick → MAVLink RC_CHANNELS_OVERRIDE for PX4 SITL.

Reads a joystick via pygame and sends RC_CHANNELS_OVERRIDE at a fixed rate.

Airframe modes (--airframe):
  quad  (default) — Detects arm/disarm stick gesture, snaps to exact PWM values
                    so PX4's internal detection always receives clean input.
  plane           — No gesture snap; arm via switch or MAVLink command.

Channel mapping (Mode 2):
  Axis 0            → CH1  Roll/Aileron      (centred)
  Axis 1            → CH2  Pitch/Elevator    (centred, inverted)
  Axis 2            → CH3  Throttle          (full-range)
  Axis 3            → CH4  Yaw/Rudder        (centred)
  Button 6 / 7      → CH5  3-pos: btn7=1000 (MANUAL), btn6=1500 (STAB), neither=2000 (MISSION)
  Button 4 / 5      → CH6  3-pos: btn5=1000 (low),    btn4=1500 (mid),  neither=2000 (high)
  Button 0          → CH7  (2000 pressed, 1000 released)
  Button 1          → CH8  (2000 pressed, 1000 released)

Quad arm gesture  (PX4 stick gesture): thr min + yaw full-right, held for COM_RC_ARM_HYST.
Quad disarm gesture:                    thr min + yaw full-left,  held for COM_RC_ARM_HYST.

Usage:
    python3 rc_joystick.py
    python3 rc_joystick.py --airframe plane
    python3 rc_joystick.py --connect udp:127.0.0.1:14540
    python3 rc_joystick.py --connect /dev/ttyUSB0 --baud 115200
    python3 rc_joystick.py --joy 1 --rate 20
"""

import argparse
import signal
import sys
import time

try:
    import pygame
except ImportError:
    print("pygame not installed — run: pip3 install pygame")
    sys.exit(1)

try:
    from pymavlink import mavutil
except ImportError:
    print("pymavlink not installed — run: pip3 install pymavlink")
    sys.exit(1)

# ── Gesture intent thresholds ────────────────────────────────────────────────
# Tuned to actual joystick output range (PX4 uses stricter values internally).
# When ALL conditions are met the script snaps RC to exact gesture values so
# PX4's own detection (thr<-0.8, yaw>0.9, |roll|<0.1, |pitch|<0.1) always passes.

GESTURE_THR   = -0.74   # throttle must be below this  (PX4: -0.80)
GESTURE_YAW   =  0.74   # |yaw| must exceed this       (PX4:  0.90)
GESTURE_ROLL  =  0.85   # |roll|  must be below this   (PX4:  0.10)
GESTURE_PITCH =  0.85   # |pitch| must be below this   (PX4:  0.10)

# ── PWM helpers ──────────────────────────────────────────────────────────────

def _axis_pwm(v: float, invert: bool = False) -> int:
    if invert:
        v = -v
    return int(max(1000, min(2000, 1500 + v * 500)))


def _thr_pwm(v: float) -> int:
    return int(max(1000, min(2000, 1000 + (v + 1.0) * 500)))


def _btn_pwm(joy: "pygame.joystick.Joystick", *indices) -> int:
    n = joy.get_numbuttons()
    return 2000 if any(i < n and joy.get_button(i) for i in indices) else 1000


def _normalize(pwm: int, min_us: int = 1000, trim_us: int = 1500, max_us: int = 2000) -> float:
    if pwm <= trim_us:
        return max(-1.0, (pwm - trim_us) / float(trim_us - min_us))
    return min(+1.0, (pwm - trim_us) / float(max_us - trim_us))


# ── Channel reader ───────────────────────────────────────────────────────────

def read_channels(joy: "pygame.joystick.Joystick") -> list[int]:
    n_ax  = joy.get_numaxes()
    n_btn = joy.get_numbuttons()

    def ax(i, inv=False):
        return _axis_pwm(joy.get_axis(i), inv) if i < n_ax else 1500

    ch1 = ax(0)
    ch2 = ax(1, inv=True)
    ch3 = _thr_pwm(joy.get_axis(2)) if n_ax > 2 else 1000
    ch4 = ax(3)

    # CH5: btn7=1000 (MANUAL), btn6=1500 (STAB), neither=2000 (MISSION)
    btn6 = 6 < n_btn and joy.get_button(6)
    btn7 = 7 < n_btn and joy.get_button(7)
    ch5  = 1000 if btn7 else (1500 if btn6 else 2000)

    # CH6: btn5=1000 (low), btn4=1500 (mid), neither=2000 (high)
    btn4 = 4 < n_btn and joy.get_button(4)
    btn5 = 5 < n_btn and joy.get_button(5)
    ch6  = 1000 if btn5 else (1500 if btn4 else 2000)

    ch7 = _btn_pwm(joy, 0)
    ch8 = _btn_pwm(joy, 1)

    return [ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8]


# ── MAVLink send ─────────────────────────────────────────────────────────────

def send_override(master, channels: list[int]):
    padded = channels[:8] + [0] * 10   # 0 = "do not override" per MAVLink spec
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *padded,
    )


def release_override(master):
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *([0] * 18),
    )


# ── Gesture snap ─────────────────────────────────────────────────────────────

def apply_gesture_snap(channels: list[int]) -> tuple[list[int], str]:
    """Detect arm/disarm intent and snap to exact gesture values for PX4."""
    ch1, ch2, ch3, ch4, ch5 = channels[:5]
    n1, n2, n3, n4 = _normalize(ch1), _normalize(ch2), _normalize(ch3), _normalize(ch4)

    thr_ok   = n3 < GESTURE_THR
    roll_ok  = abs(n1) < GESTURE_ROLL
    pitch_ok = abs(n2) < GESTURE_PITCH
    mode_ok  = ch5 <= 1100

    if thr_ok and n4 >  GESTURE_YAW and roll_ok and pitch_ok and mode_ok:
        return [1500, 1500, 1000, 2000, 1000] + channels[5:], "ARM"
    if thr_ok and n4 < -GESTURE_YAW and roll_ok and pitch_ok and mode_ok:
        return [1500, 1500, 1000, 1000, 1000] + channels[5:], "DISARM"
    return channels, ""


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Joystick → MAVLink RC_CHANNELS_OVERRIDE for PX4 SITL"
    )
    parser.add_argument("--connect", default="udp:127.0.0.1:14540",
                        help="MAVLink connection string (default: udp:127.0.0.1:14540)")
    parser.add_argument("--baud",    type=int,   default=57600)
    parser.add_argument("--joy",     type=int,   default=0,
                        help="Joystick device index (default: 0)")
    parser.add_argument("--rate",      type=float, default=10.0,
                        help="RC send rate in Hz (default: 10)")
    parser.add_argument("--airframe",  choices=["quad", "plane"], default="quad",
                        help="Airframe type: quad (gesture arm) or plane (no gesture)")
    args = parser.parse_args()

    print(f"[MAV] Connecting to {args.connect} ...")
    master = mavutil.mavlink_connection(args.connect, baud=args.baud)
    master.wait_heartbeat()
    master.mav.srcSystem = 255
    print(f"[MAV] Heartbeat — sysid={master.target_system} compid={master.target_component}")
    print(f"[CFG] Airframe: {args.airframe}")

    pygame.init()
    pygame.joystick.init()
    count = pygame.joystick.get_count()
    if count == 0:
        print("[JOY] No joystick detected")
        sys.exit(1)
    if args.joy >= count:
        print(f"[JOY] Index {args.joy} out of range — found {count} device(s):")
        for i in range(count):
            j = pygame.joystick.Joystick(i); j.init()
            print(f"  [{i}] {j.get_name()}")
        sys.exit(1)

    joy = pygame.joystick.Joystick(args.joy)
    joy.init()
    print(f"[JOY] [{args.joy}] {joy.get_name()}  "
          f"axes={joy.get_numaxes()}  buttons={joy.get_numbuttons()}")

    interval     = 1.0 / args.rate
    running      = True
    prev_gesture = ""

    def _shutdown(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        while running:
            t0 = time.monotonic()

            pygame.event.pump()
            channels = read_channels(joy)

            if args.airframe == "quad":
                channels, gesture = apply_gesture_snap(channels)
                if gesture != prev_gesture:
                    if gesture:
                        print(f"[ARM] *** {gesture} gesture detected — sending exact values, hold 3s ***")
                    else:
                        print(f"[ARM] Gesture released")
                    prev_gesture = gesture

            send_override(master, channels)

            elapsed = time.monotonic() - t0
            rem = interval - elapsed
            if rem > 0:
                time.sleep(rem)

    finally:
        release_override(master)
        print("\n[MAV] RC override released")
        joy.quit()
        pygame.quit()
        print("[JOY] Closed")


if __name__ == "__main__":
    main()
