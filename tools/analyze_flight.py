#!/usr/bin/env python3
"""
analyze_flight.py — Analyze a recorded flight session and suggest param changes.

Reads flight.csv from a session directory, computes oscillation/stability metrics
per flight phase, and prints targeted parameter suggestions for PX4 or ArduCopter.

Usage:
    python3 tools/analyze_flight.py                          # latest session
    python3 tools/analyze_flight.py logs/sessions/2026-05-19_12-30-00
    python3 tools/analyze_flight.py --list                   # list all sessions
    python3 tools/analyze_flight.py --autopilot px4          # force autopilot type
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

SESSIONS_DIR = Path(__file__).resolve().parents[1] / "logs" / "sessions"

# ── Thresholds ────────────────────────────────────────────────────────────────

# Rate oscillation (std dev rad/s while armed)
THR_RATE_OSC_WARN  = 0.15   # yellow
THR_RATE_OSC_BAD   = 0.35   # red

# Angle oscillation (std dev degrees while armed)
THR_ANGLE_OSC_WARN = 8.0
THR_ANGLE_OSC_BAD  = 20.0

# Max tilt (max |roll| or |pitch| degrees)
THR_TILT_WARN      = 25.0
THR_TILT_BAD       = 45.0

# Altitude stability (std dev m/s of vz while armed+hovering)
THR_VZ_WARN        = 0.8
THR_VZ_BAD         = 2.0

# Horizontal velocity noise (std dev m/s in position modes)
THR_VXY_WARN       = 0.4
THR_VXY_BAD        = 1.0

# EKF bad ratio (fraction of armed rows with ekf_ok==0)
THR_EKF_BAD        = 0.05


# ── Data loading ─────────────────────────────────────────────────────────────

@dataclass
class Row:
    time_s:        float
    armed:         bool
    mode:          str
    roll_deg:      float
    pitch_deg:     float
    yaw_deg:       float
    rollrate_rads: float
    pitchrate_rads:float
    yawrate_rads:  float
    alt_rel_m:     float
    vx_ms:         float
    vy_ms:         float
    vz_ms:         float
    speed_ms:      float
    throttle_pct:  float
    ekf_ok:        int
    ekf_vvar:      float
    ekf_hvar:      float
    wp_dist_m:     float


def _f(s: str) -> float:
    try:
        v = float(s)
        return v if math.isfinite(v) else float("nan")
    except (ValueError, TypeError):
        return float("nan")


def load_csv(path: Path) -> List[Row]:
    rows = []
    with open(path, newline="") as fh:
        reader = csv.DictReader(fh)
        for r in reader:
            rows.append(Row(
                time_s        = _f(r.get("time_s", "")),
                armed         = r.get("armed", "0") == "1",
                mode          = r.get("mode", ""),
                roll_deg      = _f(r.get("roll_deg", "")),
                pitch_deg     = _f(r.get("pitch_deg", "")),
                yaw_deg       = _f(r.get("yaw_deg", "")),
                rollrate_rads = _f(r.get("rollrate_rads", "")),
                pitchrate_rads= _f(r.get("pitchrate_rads", "")),
                yawrate_rads  = _f(r.get("yawrate_rads", "")),
                alt_rel_m     = _f(r.get("alt_rel_m", "")),
                vx_ms         = _f(r.get("vx_ms", "")),
                vy_ms         = _f(r.get("vy_ms", "")),
                vz_ms         = _f(r.get("vz_ms", "")),
                speed_ms      = _f(r.get("speed_ms", "")),
                throttle_pct  = _f(r.get("throttle_pct", "")),
                ekf_ok        = int(_f(r.get("ekf_ok", "1")) or 1),
                ekf_vvar      = _f(r.get("ekf_vvar", "")),
                ekf_hvar      = _f(r.get("ekf_hvar", "")),
                wp_dist_m     = _f(r.get("wp_dist_m", "")),
            ))
    return rows


# ── Statistics helpers ────────────────────────────────────────────────────────

def _vals(data: List[float]) -> List[float]:
    return [v for v in data if math.isfinite(v)]


def mean(data: List[float]) -> float:
    v = _vals(data)
    return sum(v) / len(v) if v else float("nan")


def std(data: List[float]) -> float:
    v = _vals(data)
    if len(v) < 2:
        return float("nan")
    m = sum(v) / len(v)
    return math.sqrt(sum((x - m) ** 2 for x in v) / len(v))


def rms(data: List[float]) -> float:
    v = _vals(data)
    return math.sqrt(sum(x * x for x in v) / len(v)) if v else float("nan")


def peak(data: List[float]) -> float:
    v = _vals(data)
    return max(abs(x) for x in v) if v else float("nan")


# ── Suggestion engine ─────────────────────────────────────────────────────────

@dataclass
class Issue:
    severity: str          # "WARN" or "FAIL"
    category: str
    detail:   str
    px4_fixes:  List[str] = field(default_factory=list)
    ardu_fixes: List[str] = field(default_factory=list)


def _pct_change(current: float, pct: float) -> float:
    return round(current * (1 + pct / 100), 4)


def suggest_rate_osc(roll_std: float, pitch_std: float,
                     current_px4_rate_p: float = 0.04,
                     current_ardu_rate_p: float = 0.07) -> Optional[Issue]:
    worst = max(roll_std, pitch_std)
    if worst < THR_RATE_OSC_WARN:
        return None
    sev = "FAIL" if worst >= THR_RATE_OSC_BAD else "WARN"
    factor = -30 if worst >= THR_RATE_OSC_BAD else -20
    new_px4  = _pct_change(current_px4_rate_p,  factor)
    new_ardu = _pct_change(current_ardu_rate_p, factor)
    return Issue(
        severity=sev,
        category="RATE OSCILLATION",
        detail=f"roll_rate σ={roll_std:.3f} rad/s  pitch_rate σ={pitch_std:.3f} rad/s  (threshold={THR_RATE_OSC_WARN})",
        px4_fixes=[
            f"MC_ROLLRATE_P:  {current_px4_rate_p:.4f} → {new_px4:.4f}  ({factor}%)",
            f"MC_PITCHRATE_P: {current_px4_rate_p:.4f} → {new_px4:.4f}  ({factor}%)",
            f"MC_ROLLRATE_I:  reduce proportionally",
            f"MC_ROLLRATE_D:  reduce or zero",
        ],
        ardu_fixes=[
            f"ATC_RAT_RLL_P:  {current_ardu_rate_p:.4f} → {new_ardu:.4f}  ({factor}%)",
            f"ATC_RAT_PIT_P:  {current_ardu_rate_p:.4f} → {new_ardu:.4f}  ({factor}%)",
            f"ATC_RAT_RLL_I:  reduce to match P",
            f"ATC_RAT_RLL_D:  reduce or zero",
        ],
    )


def suggest_angle_osc(roll_std: float, pitch_std: float,
                      current_px4_ang_p: float = 2.0,
                      current_ardu_ang_p: float = 4.5) -> Optional[Issue]:
    worst = max(roll_std, pitch_std)
    if worst < THR_ANGLE_OSC_WARN:
        return None
    sev = "FAIL" if worst >= THR_ANGLE_OSC_BAD else "WARN"
    factor = -25 if worst >= THR_ANGLE_OSC_BAD else -15
    new_px4  = _pct_change(current_px4_ang_p,  factor)
    new_ardu = _pct_change(current_ardu_ang_p, factor)
    return Issue(
        severity=sev,
        category="ANGLE OSCILLATION",
        detail=f"roll σ={roll_std:.1f}°  pitch σ={pitch_std:.1f}°  (threshold={THR_ANGLE_OSC_WARN}°)",
        px4_fixes=[
            f"MC_ROLL_P:  {current_px4_ang_p:.2f} → {new_px4:.2f}  ({factor}%)",
            f"MC_PITCH_P: {current_px4_ang_p:.2f} → {new_px4:.2f}  ({factor}%)",
        ],
        ardu_fixes=[
            f"ATC_ANG_RLL_P: {current_ardu_ang_p:.2f} → {new_ardu:.2f}  ({factor}%)",
            f"ATC_ANG_PIT_P: {current_ardu_ang_p:.2f} → {new_ardu:.2f}  ({factor}%)",
        ],
    )


def suggest_tilt(max_tilt: float) -> Optional[Issue]:
    if max_tilt < THR_TILT_WARN:
        return None
    sev = "FAIL" if max_tilt >= THR_TILT_BAD else "WARN"
    return Issue(
        severity=sev,
        category="EXCESSIVE TILT",
        detail=f"max tilt={max_tilt:.1f}°  (threshold={THR_TILT_WARN}°)",
        px4_fixes=[
            "Check CA_ROTOR_CT — if too low, mixer saturates and loses attitude control",
            "Verify motor directions and KM signs match physical wiring",
            "Reduce MC_ROLLRATE_P / MC_PITCHRATE_P by 30%",
        ],
        ardu_fixes=[
            "Check MOT_THST_EXPO — linearize thrust response",
            "Verify FRAME_TYPE and motor order match physical wiring",
            "Reduce ATC_RAT_RLL_P / ATC_RAT_PIT_P by 30%",
        ],
    )


def suggest_alt_bobbing(vz_std: float,
                        current_px4_z_p: float = 1.5,
                        current_ardu_z_p: float = 5.0) -> Optional[Issue]:
    if vz_std < THR_VZ_WARN:
        return None
    sev = "FAIL" if vz_std >= THR_VZ_BAD else "WARN"
    factor = -25 if vz_std >= THR_VZ_BAD else -15
    new_px4  = _pct_change(current_px4_z_p,  factor)
    new_ardu = _pct_change(current_ardu_z_p, factor)
    return Issue(
        severity=sev,
        category="ALTITUDE BOBBING",
        detail=f"vz σ={vz_std:.3f} m/s  (threshold={THR_VZ_WARN} m/s)",
        px4_fixes=[
            f"MPC_Z_VEL_P_ACC: {current_px4_z_p:.2f} → {new_px4:.2f}  ({factor}%)",
            f"MPC_Z_VEL_I_ACC: reduce proportionally",
            f"MPC_Z_P:         reduce if still unstable",
        ],
        ardu_fixes=[
            f"PSC_VELZ_P:      {current_ardu_z_p:.2f} → {new_ardu:.2f}  ({factor}%)",
            f"PILOT_ACCEL_Z:   reduce if still unstable",
        ],
    )


def suggest_pos_drift(vxy_std: float,
                      current_px4_xy_p: float = 0.6,
                      current_ardu_xy_p: float = 2.0) -> Optional[Issue]:
    if vxy_std < THR_VXY_WARN:
        return None
    sev = "FAIL" if vxy_std >= THR_VXY_BAD else "WARN"
    factor = -25 if vxy_std >= THR_VXY_BAD else -15
    new_px4  = _pct_change(current_px4_xy_p,  factor)
    new_ardu = _pct_change(current_ardu_xy_p, factor)
    return Issue(
        severity=sev,
        category="POSITION INSTABILITY",
        detail=f"horizontal speed σ={vxy_std:.3f} m/s in position mode  (threshold={THR_VXY_WARN} m/s)",
        px4_fixes=[
            f"MPC_XY_VEL_P_ACC: {current_px4_xy_p:.2f} → {new_px4:.2f}  ({factor}%)",
            f"MPC_XY_VEL_I_ACC: reduce proportionally",
            f"MPC_XY_P:         reduce if still unstable",
        ],
        ardu_fixes=[
            f"PSC_VELXY_P:      {current_ardu_xy_p:.2f} → {new_ardu:.2f}  ({factor}%)",
            f"LOIT_SPEED:       reduce cruise speed",
        ],
    )


def suggest_ekf(bad_frac: float, ekf_vvar_mean: float) -> Optional[Issue]:
    if bad_frac < THR_EKF_BAD:
        return None
    return Issue(
        severity="FAIL",
        category="EKF UNHEALTHY",
        detail=f"ekf_ok=0 on {bad_frac*100:.1f}% of armed rows  vvar_mean={ekf_vvar_mean:.3f}",
        px4_fixes=[
            "Check EKF2_ABL_LIM — increase if 'High Accelerometer Bias' persists",
            "Set EKF2_BARO_CTRL=1 and CAL_BARO1_PRIO=0",
            "Verify SENS_EN_BAROSIM=1 and SENS_EN_GPSSIM=1",
        ],
        ardu_fixes=[
            "Set EK3_SRC1_VELZ=0 if GPS not active (main EKF3 unhealthy cause)",
            "Set COMPASS_USE=0 for X-Plane HIL (compass innovations fail)",
            "Increase EK3_CHECK_SCALE to 300, EK3_ERR_THRESH to 0.8",
        ],
    )


# ── Analysis ──────────────────────────────────────────────────────────────────

POSITION_MODES = {
    # PX4
    "POSCTL", "AUTO/MISSION", "AUTO/LOITER", "AUTO/TAKEOFF",
    "AUTO/LAND", "AUTO/RTL",
    # ArduCopter
    "LOITER", "AUTO", "POSHOLD", "GUIDED", "RTL",
}

HOVER_MODES = POSITION_MODES | {"ALTCTL", "ALT_HOLD"}


def run_analysis(rows: List[Row], autopilot: str) -> List[Issue]:
    armed = [r for r in rows if r.armed]
    if not armed:
        print("WARNING: no armed rows found — nothing to analyse.")
        return []

    pos_rows  = [r for r in armed if r.mode in POSITION_MODES]
    hover_rows = [r for r in armed if r.mode in HOVER_MODES]

    roll_std  = std([r.roll_deg      for r in armed])
    pitch_std = std([r.pitch_deg     for r in armed])
    rr_std    = std([r.rollrate_rads  for r in armed])
    pr_std    = std([r.pitchrate_rads for r in armed])
    max_tilt  = peak([max(abs(r.roll_deg), abs(r.pitch_deg)) for r in armed])

    vz_std    = std([r.vz_ms for r in hover_rows if math.isfinite(r.vz_ms)])
    vxy_vals  = [math.hypot(r.vx_ms, r.vy_ms) for r in pos_rows
                 if math.isfinite(r.vx_ms) and math.isfinite(r.vy_ms)]
    vxy_std   = std(vxy_vals)

    ekf_bad   = sum(1 for r in armed if r.ekf_ok == 0)
    ekf_frac  = ekf_bad / len(armed)
    ekf_vvar_vals = [r.ekf_vvar for r in armed if math.isfinite(r.ekf_vvar)]
    ekf_vvar_mean = mean(ekf_vvar_vals)

    issues: List[Issue] = []

    i = suggest_rate_osc(rr_std, pr_std)
    if i: issues.append(i)

    i = suggest_angle_osc(roll_std, pitch_std)
    if i: issues.append(i)

    i = suggest_tilt(max_tilt)
    if i: issues.append(i)

    if math.isfinite(vz_std):
        i = suggest_alt_bobbing(vz_std)
        if i: issues.append(i)

    if math.isfinite(vxy_std):
        i = suggest_pos_drift(vxy_std)
        if i: issues.append(i)

    i = suggest_ekf(ekf_frac, ekf_vvar_mean)
    if i: issues.append(i)

    return issues


# ── Report ────────────────────────────────────────────────────────────────────

SEV_COLOR = {"WARN": "\033[33m", "FAIL": "\033[31m", "OK": "\033[32m"}
RESET = "\033[0m"


def _sev(s: str) -> str:
    return f"{SEV_COLOR.get(s, '')}{s}{RESET}"


def print_summary(rows: List[Row], autopilot: str):
    armed = [r for r in rows if r.armed]
    modes = {}
    for r in armed:
        modes[r.mode] = modes.get(r.mode, 0) + 1
    total = len(rows)
    duration = rows[-1].time_s - rows[0].time_s if len(rows) > 1 else 0

    print(f"\n{'─'*60}")
    print(f"  FLIGHT SUMMARY  |  {autopilot}  |  {total} rows  |  {duration:.0f}s")
    print(f"{'─'*60}")
    print(f"  Armed rows : {len(armed)}  ({100*len(armed)/max(total,1):.0f}%)")
    if modes:
        print("  Modes (armed):")
        for m, c in sorted(modes.items(), key=lambda x: -x[1]):
            print(f"    {m:<20} {c} rows")

    if armed:
        roll_vals  = [r.roll_deg  for r in armed if math.isfinite(r.roll_deg)]
        pitch_vals = [r.pitch_deg for r in armed if math.isfinite(r.pitch_deg)]
        rr_vals    = [r.rollrate_rads  for r in armed if math.isfinite(r.rollrate_rads)]
        pr_vals    = [r.pitchrate_rads for r in armed if math.isfinite(r.pitchrate_rads)]
        vz_vals    = [r.vz_ms for r in armed if math.isfinite(r.vz_ms)]

        print(f"\n  ATTITUDE (armed)")
        print(f"    roll  mean={mean(roll_vals):+6.2f}°  σ={std(roll_vals):.2f}°  peak=±{peak(roll_vals):.1f}°")
        print(f"    pitch mean={mean(pitch_vals):+6.2f}°  σ={std(pitch_vals):.2f}°  peak=±{peak(pitch_vals):.1f}°")

        print(f"\n  RATES (armed)")
        print(f"    rollrate  σ={std(rr_vals):.4f} rad/s  rms={rms(rr_vals):.4f} rad/s")
        print(f"    pitchrate σ={std(pr_vals):.4f} rad/s  rms={rms(pr_vals):.4f} rad/s")

        print(f"\n  ALTITUDE (armed)")
        print(f"    vz  σ={std(vz_vals):.3f} m/s  peak={peak(vz_vals):.2f} m/s")
    print()


def print_issues(issues: List[Issue], autopilot: str):
    is_px4 = "px4" in autopilot.lower()
    if not issues:
        print(f"  {_sev('OK')}  No stability issues detected.\n")
        return

    for issue in issues:
        print(f"  [{_sev(issue.severity)}] {issue.category}")
        print(f"       {issue.detail}")
        fixes = issue.px4_fixes if is_px4 else issue.ardu_fixes
        if fixes:
            print(f"       Suggested fix ({autopilot}):")
            for fix in fixes:
                print(f"         → {fix}")
        print()


# ── Session selection ─────────────────────────────────────────────────────────

def latest_session(base: Path) -> Optional[Path]:
    if not base.exists():
        return None
    sessions = sorted(base.iterdir(), reverse=True)
    for s in sessions:
        if (s / "flight.csv").exists():
            return s
    return None


def list_sessions(base: Path):
    if not base.exists():
        print(f"No sessions found in {base}")
        return
    sessions = sorted(base.iterdir(), reverse=True)
    print(f"Sessions in {base}:")
    for s in sessions:
        csv_path = s / "flight.csv"
        meta_path = s / "metadata.json"
        if not csv_path.exists():
            continue
        rows = sum(1 for _ in open(csv_path)) - 1
        ap = "?"
        if meta_path.exists():
            try:
                ap = json.loads(meta_path.read_text()).get("autopilot", "?")
            except Exception:
                pass
        print(f"  {s.name}  {rows} rows  [{ap}]")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", nargs="?", default=None,
                    help="Session directory (default: latest)")
    ap.add_argument("--base",       default=str(SESSIONS_DIR),
                    help="Parent directory of sessions")
    ap.add_argument("--list",       action="store_true", help="List all sessions and exit")
    ap.add_argument("--autopilot",  default=None, choices=["px4", "arducopter"],
                    help="Force autopilot type (default: auto-detect from metadata)")
    args = ap.parse_args()

    base = Path(args.base)

    if args.list:
        list_sessions(base)
        return

    # Resolve session path
    if args.session:
        session_path = Path(args.session)
        if not session_path.is_absolute():
            session_path = Path.cwd() / session_path
    else:
        session_path = latest_session(base)
        if session_path is None:
            print(f"ERROR: no sessions found in {base}. Run record_flight.py first.")
            sys.exit(1)
        print(f"Using latest session: {session_path.name}")

    csv_path  = session_path / "flight.csv"
    meta_path = session_path / "metadata.json"

    if not csv_path.exists():
        print(f"ERROR: {csv_path} not found")
        sys.exit(1)

    # Detect autopilot
    autopilot = args.autopilot or "PX4"
    if meta_path.exists():
        try:
            autopilot = json.loads(meta_path.read_text()).get("autopilot", autopilot)
        except Exception:
            pass
    if args.autopilot:
        autopilot = args.autopilot.upper()

    # Load and analyse
    rows   = load_csv(csv_path)
    issues = run_analysis(rows, autopilot)

    print_summary(rows, autopilot)
    print(f"{'─'*60}")
    print(f"  ISSUES & SUGGESTIONS  ({len(issues)} found)")
    print(f"{'─'*60}")
    print_issues(issues, autopilot)

    # Save analysis report
    report_path = session_path / "analysis.txt"
    with open(report_path, "w") as fh:
        fh.write(f"Analysis: {session_path.name}\n")
        fh.write(f"Autopilot: {autopilot}\n")
        fh.write(f"Rows: {len(rows)}\n\n")
        for issue in issues:
            is_px4 = "px4" in autopilot.lower()
            fh.write(f"[{issue.severity}] {issue.category}\n")
            fh.write(f"  {issue.detail}\n")
            fixes = issue.px4_fixes if is_px4 else issue.ardu_fixes
            for fix in fixes:
                fh.write(f"  → {fix}\n")
            fh.write("\n")
    print(f"Report saved → {report_path}")


if __name__ == "__main__":
    main()
