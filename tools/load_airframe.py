#!/usr/bin/env python3
"""
Load a PX4 airframe parameter file into a running PX4 SITL instance via MAVLink.
Fetches all current param types first, then applies 'param set-default' entries.

Usage:
    python3 load_airframe.py <airframe_file> [--host 127.0.0.1] [--port 14540]
"""

import sys
import re
import time
import struct
import argparse
from pymavlink import mavutil

REAL32 = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
INT32  = mavutil.mavlink.MAV_PARAM_TYPE_INT32


def to_wire_float(value: float, ptype: int) -> float:
    """MAVLink encodes all param values as float on the wire.
    For integer types PX4 reinterprets the raw bytes as int, so we must
    bit-cast the integer into a float before sending — not just cast with float()."""
    if ptype == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
        return struct.unpack('f', struct.pack('i', int(round(value))))[0]
    if ptype == mavutil.mavlink.MAV_PARAM_TYPE_UINT32:
        return struct.unpack('f', struct.pack('I', int(round(value))))[0]
    return float(value)  # REAL32 — pass through unchanged


def wait_heartbeat(conn, timeout=30):
    print(f"Waiting for heartbeat...")
    conn.wait_heartbeat(timeout=timeout)
    print(f"  Connected — sysid={conn.target_system}\n")


def fetch_all_param_types(conn, timeout=30):
    """Request full param list and record each param's MAVLink type."""
    print("Fetching all param types from PX4...")
    conn.mav.param_request_list_send(conn.target_system, conn.target_component)
    types = {}
    expected = None
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg is None:
            continue
        name = msg.param_id.rstrip("\x00")
        types[name] = msg.param_type
        if expected is None:
            expected = msg.param_count
        if len(types) >= expected:
            break
        deadline = time.time() + 3.0  # extend on activity
    print(f"  Got {len(types)} param types\n")
    return types


def parse_airframe(path):
    """Extract (name, value_str) pairs from 'param set-default NAME VALUE' lines."""
    params = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line.startswith("#") or line.startswith(";"):
                continue
            m = re.match(r"param\s+set(?:-default)?\s+(\S+)\s+(\S+)", line)
            if m:
                params.append((m.group(1), m.group(2)))
    return params


def set_param(conn, name, value, ptype, retries=3, ack_timeout=3.0):
    wire_value = to_wire_float(value, ptype)
    for attempt in range(1, retries + 1):
        conn.mav.param_set_send(
            conn.target_system, conn.target_component,
            name.encode(), wire_value, ptype)
        deadline = time.time() + ack_timeout
        while time.time() < deadline:
            msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg and msg.param_id.rstrip("\x00") == name:
                return msg.param_value
        if attempt < retries:
            print(f"    [retry {attempt}] {name}")
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("airframe", help="Path to airframe file")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=14540)
    args = ap.parse_args()

    conn = mavutil.mavlink_connection(
        f"udp:{args.host}:{args.port}", source_system=254)
    wait_heartbeat(conn)

    # Learn actual param types from PX4
    known_types = fetch_all_param_types(conn)

    # Parse airframe file
    entries = parse_airframe(args.airframe)
    print(f"Loading {len(entries)} params from {args.airframe}\n")

    ok = fail = skip = 0
    for name, val_str in entries:
        # Determine type: prefer what PX4 told us; fall back to heuristic
        if name in known_types:
            ptype = known_types[name]
        elif "." in val_str:
            ptype = REAL32
        else:
            ptype = INT32

        try:
            value = float(val_str)
        except ValueError:
            print(f"  {name:35s}  SKIP (unparseable value: {val_str!r})")
            skip += 1
            continue

        acked = set_param(conn, name, value, ptype)
        if acked is not None:
            print(f"  {name:35s}  {value:>12.4f}  OK")
            ok += 1
        else:
            print(f"  {name:35s}  FAILED")
            fail += 1

    print(f"\nDone: {ok} set, {fail} failed, {skip} skipped.")
    conn.close()


if __name__ == "__main__":
    main()
