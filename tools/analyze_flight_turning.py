#!/usr/bin/env python3
"""
analyze_flight_turning.py — Diagnose persistent turning / yaw drift from a px4xplane
flight log recorded by record_flight.py.

Outputs:
  • Console report: statistics, root-cause analysis, parameter recommendations
  • Optional plots (--plot): yaw rate, heading error, motor imbalance time-series

Usage:
    python3 tools/analyze_flight_turning.py flightlog_20240601_120000.csv
    python3 tools/analyze_flight_turning.py flightlog_20240601_120000.csv --plot
    python3 tools/analyze_flight_turning.py flightlog_20240601_120000.csv --airframe ehang184

Requirements:
    pip install pandas numpy
    pip install matplotlib   (optional, for --plot)
"""

import argparse
import math
import os
import sys

try:
    import pandas as pd
    import numpy as np
except ImportError:
    print("ERROR: pandas/numpy not installed.  Run: pip install pandas numpy")
    sys.exit(1)

# ── PX4 mode constants ─────────────────────────────────────────────────────────
MAIN_MODE_AUTO       = 4
SUB_MODE_AUTO_MISSION = 4   # AUTO_MISSION = 4 in PX4 sub-mode enum
SUB_MODE_AUTO_LOITER  = 3
MAV_MODE_FLAG_ARMED   = 128  # bit 7

# Yaw rate threshold for "actively spinning" (rad/s)
SPIN_THRESHOLD = 0.05        # ~3 deg/s

# Heading error threshold for "nav off-course"
HEADING_ERROR_THRESHOLD_DEG = 15.0

# Motor imbalance threshold (normalised difference, 0-1 scale)
MOTOR_IMBALANCE_THRESHOLD = 0.05


# ── helpers ────────────────────────────────────────────────────────────────────
def wrap_deg(a):
    """Wrap angle to [-180, 180]."""
    return ((a + 180) % 360) - 180


def describe(arr, label, unit=""):
    if len(arr) == 0:
        return f"  {label}: no data"
    return (f"  {label:35s}: "
            f"mean={np.mean(arr):+.4f}  "
            f"std={np.std(arr):.4f}  "
            f"min={np.min(arr):+.4f}  "
            f"max={np.max(arr):+.4f}  "
            f"p5={np.percentile(arr,5):+.4f}  "
            f"p95={np.percentile(arr,95):+.4f}"
            + (f"  [{unit}]" if unit else ""))


def heading_error(actual, target):
    """Signed error (actual - target) wrapped to [-180, 180]."""
    return wrap_deg(actual - target)


# ── severity classification ────────────────────────────────────────────────────
# RUNAWAY  : |mean| > 1.0 rad/s (>57 deg/s) — positive-feedback, controller amplifying spin
# SEVERE   : |mean| > 0.3 rad/s (>17 deg/s) — strong bias, integral needed
# DRIFT    : |mean| > SPIN_THRESHOLD          — mild steady-state bias
# NORMAL   : no significant spin
RUNAWAY_THRESHOLD = 1.0   # rad/s
SEVERE_THRESHOLD  = 0.3   # rad/s


def classify_spin(mean_yr):
    a = abs(mean_yr)
    if a >= RUNAWAY_THRESHOLD: return "RUNAWAY"
    if a >= SEVERE_THRESHOLD:  return "SEVERE"
    if a >= SPIN_THRESHOLD:    return "DRIFT"
    return "NORMAL"


# ── recommendations engine ─────────────────────────────────────────────────────
def recommend(df_mission, df_hover, current_params, airframe):
    recs = []
    warnings = []

    # ── 1. Yaw rate bias ──────────────────────────────────────────────────────
    if len(df_hover) > 20:
        mean_yaw_rate = df_hover["yawrate_rads"].mean()
    elif len(df_mission) > 20:
        mean_yaw_rate = df_mission["yawrate_rads"].mean()
    else:
        mean_yaw_rate = 0.0

    is_ccw     = mean_yaw_rate < -SPIN_THRESHOLD
    is_cw      = mean_yaw_rate >  SPIN_THRESHOLD
    spin_class = classify_spin(mean_yaw_rate)

    if spin_class != "NORMAL":
        direction = "CCW (negative)" if is_ccw else "CW (positive)"
        label = {"RUNAWAY": "RUNAWAY SPIN", "SEVERE": "SEVERE YAW DRIFT",
                 "DRIFT": "PERSISTENT YAW DRIFT"}.get(spin_class, "YAW DRIFT")
        warnings.append(
            f"{label}: mean yaw rate = {math.degrees(mean_yaw_rate):.1f} deg/s "
            f"({direction}) — {spin_class}"
        )

    # ── 2. RUNAWAY: KM signs inverted (positive-feedback loop) ────────────────
    yaw_rate_max_rads = math.radians(current_params.get("MC_YAWRATE_MAX", 100.0))
    if spin_class == "RUNAWAY":
        direction_word = "CCW" if is_ccw else "CW"
        opp_word       = "CW"  if is_ccw else "CCW"
        warnings.append(
            f"SPIN EXCEEDS MC_YAWRATE_MAX "
            f"({abs(math.degrees(mean_yaw_rate)):.0f} deg/s vs "
            f"{math.degrees(yaw_rate_max_rads):.0f} deg/s limit) — "
            "controller is saturated AND likely amplifying the spin."
        )
        recs.append({
            "param":   "CA_ROTOR0/1/2/3_KM  — NEGATE ALL SIGNS",
            "current": f"{current_params.get('CA_ROTOR0_KM'):+.2f}, {current_params.get('CA_ROTOR1_KM'):+.2f}, {current_params.get('CA_ROTOR2_KM'):+.2f}, {current_params.get('CA_ROTOR3_KM'):+.2f}",
            "suggest": f"{-current_params.get('CA_ROTOR0_KM'):+.2f}, {-current_params.get('CA_ROTOR1_KM'):+.2f}, {-current_params.get('CA_ROTOR2_KM'):+.2f}, {-current_params.get('CA_ROTOR3_KM'):+.2f}  (negate all)",
            "reason":  (
                f"RUNAWAY {direction_word} spin ({abs(math.degrees(mean_yaw_rate)):.0f} deg/s) "
                f"exceeds the controller's maximum rate. "
                "This is the signature of a positive-feedback loop: PX4 commands "
                f"{opp_word} correction but the wrong KM signs mean the actuators "
                f"create MORE {direction_word} torque instead. "
                "Step 1: negate all four CA_ROTORx_KM values and retest. "
                "If the spin reverses direction, the signs are confirmed wrong "
                "and you need to also swap the sign of the X-Plane propeller "
                "rotation in the aircraft model. "
                "Step 2 (after confirming direction): reduce |KM| from 0.05 "
                "toward 0.02–0.03 if spin is reduced but still present."
            ),
            "priority": "CRITICAL",
        })
        recs.append({
            "param":   "MC_YAWRATE_MAX",
            "current": current_params.get("MC_YAWRATE_MAX", 100.0),
            "suggest": 200.0,
            "reason":  (
                f"Observed spin ({abs(math.degrees(mean_yaw_rate)):.0f} deg/s) "
                "exceeds the current MC_YAWRATE_MAX limit. Even after fixing KM "
                "signs, raise the limit so the controller can command enough "
                "counter-torque to arrest rapid spins during recovery."
            ),
            "priority": "HIGH",
        })

    # ── 3. SEVERE / DRIFT: integral and gain checks ───────────────────────────
    if spin_class in ("SEVERE", "DRIFT"):
        yaw_i = current_params.get("MC_YAWRATE_I", 0.0)
        if yaw_i < 0.05:
            recs.append({
                "param":   "MC_YAWRATE_I",
                "current": yaw_i,
                "suggest": 0.1,
                "reason":  (
                    "Yaw integral is near zero — cannot correct a steady-state "
                    "torque disturbance.  Start at 0.1 and increase until slight "
                    "oscillation, then back off 50%."
                ),
                "priority": "HIGH",
            })

        yaw_k = current_params.get("MC_YAWRATE_K", 1.0)
        yaw_p = current_params.get("MC_YAWRATE_P", 1.0)
        effective_p = yaw_k * yaw_p
        if effective_p < 1.5:
            recs.append({
                "param":   "MC_YAWRATE_K",
                "current": yaw_k,
                "suggest": round(yaw_k * 1.5, 2),
                "reason":  (
                    f"Effective yaw-rate P = K({yaw_k}) × P({yaw_p}) = {effective_p:.2f}. "
                    "Increase MC_YAWRATE_K in 20% steps until slight oscillation, "
                    "then back off."
                ),
                "priority": "MEDIUM",
            })

        if airframe in ("ehang184", "xplane_ehang184"):
            recs.append({
                "param":   "CA_ROTOR0_KM / CA_ROTOR1_KM",
                "current": "±0.05",
                "suggest": "Verify X-Plane prop spin directions match KM signs",
                "reason":  (
                    f"{'CCW' if is_ccw else 'CW'} torque bias.  "
                    "Confirm motor 0 (FR) and motor 1 (RL) spin CCW, "
                    "motor 2 (FL) and motor 3 (RR) spin CW in X-Plane.  "
                    "If correct, try reducing |KM| by 10% (0.045) OR adding "
                    "MC_YAWRATE_I first."
                ),
                "priority": "HIGH",
            })

    # ── 4. Heading vs target bearing error ────────────────────────────────────
    if "nav_bearing_deg" in df_mission.columns and "heading_deg" in df_mission.columns:
        df_nav = df_mission[df_mission["nav_bearing_deg"] != 0].copy()
        if len(df_nav) > 10:
            df_nav["hdg_err"] = df_nav.apply(
                lambda r: heading_error(r["heading_deg"], r["nav_bearing_deg"]), axis=1)
            mean_hdg_err = df_nav["hdg_err"].mean()
            if abs(mean_hdg_err) > HEADING_ERROR_THRESHOLD_DEG:
                warnings.append(
                    f"HEADING DIVERGENCE: mean error = {mean_hdg_err:.1f} deg "
                    "from nav_bearing during mission"
                )
                if spin_class not in ("RUNAWAY",):
                    recs.append({
                        "param":   "MC_YAW_P",
                        "current": current_params.get("MC_YAW_P", 0.6),
                        "suggest": round(current_params.get("MC_YAW_P", 0.6) * 1.3, 2),
                        "reason":  (
                            f"Heading error {mean_hdg_err:.1f} deg vs nav bearing.  "
                            "Outer-loop yaw P too low.  Increase in 30% steps."
                        ),
                        "priority": "MEDIUM",
                    })

    # ── 5. MIS_YAW_ERR ────────────────────────────────────────────────────────
    mis_yaw_err = current_params.get("MIS_YAW_ERR", 12.0)
    if spin_class in ("DRIFT", "SEVERE") and mis_yaw_err > 20.0:
        recs.append({
            "param":   "MIS_YAW_ERR",
            "current": mis_yaw_err,
            "suggest": 12.0,
            "reason":  (
                f"MIS_YAW_ERR = {mis_yaw_err} deg allows large heading deviation "
                "before mission gate triggers.  12 deg is tighter."
            ),
            "priority": "LOW",
        })

    # ── 6. IMU_GYRO_RATEMAX ────────────────────────────────────────────────────
    gyro_rate = current_params.get("IMU_GYRO_RATEMAX", 800)
    if gyro_rate < 200:
        recs.append({
            "param":   "IMU_GYRO_RATEMAX",
            "current": gyro_rate,
            "suggest": 400,
            "reason":  (
                "Gyro sample rate < 200 Hz: rate-loop commands are delayed.  "
                "For large-rotor craft set ≥ 400 Hz."
            ),
            "priority": "MEDIUM",
        })

    return warnings, recs


# ── motor mapping detection ────────────────────────────────────────────────────
def detect_motor_mapping(df_armed, df_mission):
    """
    Infer motor mapping correctness purely from attitude dynamics.

    Four independent tests — none requires motor output telemetry:

      T1 – Yaw authority direction  : when heading error is +CW, does yaw rate
           go −CCW (correct) or +CW (reversed)?  Reversed → all KM signs wrong.

      T2 – Roll-Yaw coupling        : Pearson r(roll_deg, yawrate_rads).
           Should be ≈0 for correct mapping; high |r| → diagonal motor swap.

      T3 – Pitch-Yaw coupling       : Pearson r(pitch_deg, yawrate_rads).
           Should be ≈0; high |r| → front-back motor swap.

      T4 – Heading convergence      : during mission, does |heading_error|
           grow over time?  If yes, yaw authority is ineffective or reversed.

    Returns (findings, recs) — findings = [(severity, message), ...],
    recs = [recommendation_dict, ...].
    """
    findings = []
    recs     = []
    MIN_N    = 20          # minimum sample count for any test

    ref = df_armed if len(df_armed) >= MIN_N else df_mission

    # ── T0: KM / CT authority ratio (ehang184 values) ────────────────────────
    # For a quadrotor, KM/CT ≈ yaw-moment-per-unit-thrust.
    # Too small → controller has no real yaw authority.
    km_ct_ratio = 0.05 / 6.5  # ehang184 defaults
    if km_ct_ratio < 0.008:
        findings.append((
            "WARNING",
            f"LOW KM/CT RATIO: KM/CT = {km_ct_ratio:.4f} (<0.008).  "
            "Yaw moment per unit thrust is very small — the controller may "
            "not have enough authority even when gains are correct.  "
            "Consider increasing |CA_ROTORx_KM| to 0.08–0.12."
        ))

    # ── T1: Yaw authority direction ───────────────────────────────────────────
    has_nav_data = (
        "nav_bearing_deg" in df_mission.columns and
        len(df_mission) >= MIN_N and
        (df_mission["nav_bearing_deg"] != 0).sum() >= MIN_N
    )
    if not has_nav_data:
        findings.append((
            "WARNING",
            "T1 SKIPPED — no NAV_CONTROLLER_OUTPUT data in log.  "
            "Yaw authority direction test could not run.  "
            "Fix: ensure the GCS MAVLink port (14556) is active and "
            "SET_MESSAGE_INTERVAL for msg_id=62 (NAV_CONTROLLER_OUTPUT) "
            "is configured in record_flight.py."
        ))
    if len(df_mission) >= MIN_N and "nav_bearing_deg" in df_mission.columns:
        df_t1 = df_mission[df_mission["nav_bearing_deg"] != 0].copy()
        if len(df_t1) >= MIN_N:
            df_t1["hdg_err"] = df_t1.apply(
                lambda r: heading_error(r["heading_deg"], r["nav_bearing_deg"]), axis=1)
            # Only samples with a meaningful error (controller actively correcting)
            df_t1 = df_t1[df_t1["hdg_err"].abs() > 10]
            if len(df_t1) >= MIN_N:
                # Same sign = yawing in wrong direction (error growing)
                same_sign_frac = (df_t1["hdg_err"] * df_t1["yawrate_rads"] > 0).mean()
                if same_sign_frac > 0.60:
                    findings.append((
                        "CRITICAL",
                        f"YAW AUTHORITY REVERSED ({same_sign_frac*100:.0f}% of samples): "
                        "yaw rate is consistently in the WRONG direction relative to "
                        "heading error — the aircraft is spinning away from, not toward, "
                        "its target bearing.  All KM signs are likely inverted "
                        "or all motor spin directions are wrong in X-Plane."
                    ))
                    recs.append({
                        "param":   "CA_ROTOR0/1/2/3_KM  (all four)",
                        "current": "+0.05, +0.05, −0.05, −0.05",
                        "suggest": "Negate all: −0.05, −0.05, +0.05, +0.05",
                        "reason":  (
                            "Reversed yaw authority signature: the heading error grows "
                            "while the controller is active.  This is the definitive sign "
                            "that all KM values have the wrong sign (PX4 thinks CCW motors "
                            "are CW and vice-versa).  Negate all four CA_ROTORx_KM values. "
                            "Also verify the X-Plane aircraft model has FR and RL spinning "
                            "CCW and FL and RR spinning CW when viewed from above."
                        ),
                        "priority": "CRITICAL",
                    })
                elif same_sign_frac > 0.45:
                    findings.append((
                        "WARNING",
                        f"MARGINAL YAW AUTHORITY ({same_sign_frac*100:.0f}% wrong direction). "
                        "Expected < 40% for a correct mapping.  Possible partial motor "
                        "swap or KM magnitude mismatch."
                    ))

    # ── T2: Roll-Yaw coupling ─────────────────────────────────────────────────
    if len(ref) >= MIN_N:
        ry_corr = float(ref["roll_deg"].corr(ref["yawrate_rads"]))
        if not math.isnan(ry_corr):
            if abs(ry_corr) > 0.45:
                sign_desc = "positive" if ry_corr > 0 else "negative"
                if ry_corr > 0:
                    swap_desc = "FR(0)↔RR(3) or FL(2)↔RL(1)  — same-side, cross-type swap"
                    fix_desc  = (
                        "Roll-right causes yaw-right.  In the ehang184 layout, this "
                        "happens when a CCW motor and an adjacent CW motor on the same "
                        "lateral side are swapped (e.g., FR-CCW connected to RR-CW ESC "
                        "channel or vice-versa).  "
                        "Check: PWM_MAIN_FUNC1=101(FR-CCW), FUNC2=102(RL-CCW), "
                        "FUNC3=103(FL-CW), FUNC4=104(RR-CW)."
                    )
                else:
                    swap_desc = "FR(0)↔FL(2) or RL(1)↔RR(3)  — cross-side, same-row swap"
                    fix_desc  = (
                        "Roll-right causes yaw-left.  This pattern arises when the two "
                        "front motors are swapped with each other, or the two rear motors "
                        "are swapped, so roll and yaw commands fight each other.  "
                        "Verify X-Plane prop rotation directions AND PWM_MAIN_FUNC mapping."
                    )
                findings.append((
                    "HIGH",
                    f"ROLL-YAW COUPLING  r = {ry_corr:+.3f} ({sign_desc}): "
                    f"likely swap pattern — {swap_desc}."
                ))
                recs.append({
                    "param":   f"Motor swap: {swap_desc}",
                    "current": "0=FR(CCW), 1=RL(CCW), 2=FL(CW), 3=RR(CW)",
                    "suggest": "Verify and fix PWM_MAIN_FUNC1-4 and X-Plane prop directions",
                    "reason":  fix_desc,
                    "priority": "HIGH",
                })
            elif abs(ry_corr) > 0.30:
                findings.append((
                    "WARNING",
                    f"Moderate roll-yaw coupling  r = {ry_corr:+.3f}  "
                    f"(|r| > 0.30, suspicious but not conclusive)."
                ))

    # ── T3: Pitch-Yaw coupling ────────────────────────────────────────────────
    if len(ref) >= MIN_N:
        py_corr = float(ref["pitch_deg"].corr(ref["yawrate_rads"]))
        if not math.isnan(py_corr) and abs(py_corr) > 0.40:
            sign_desc = "positive" if py_corr > 0 else "negative"
            findings.append((
                "HIGH",
                f"PITCH-YAW COUPLING  r = {py_corr:+.3f} ({sign_desc}): "
                "pitching and yawing are correlated — symptom of a diagonal "
                "motor swap (e.g., FR↔RL or FL↔RR — same rotation type, "
                "different axis)."
            ))
            recs.append({
                "param":   "Motor swap: diagonal same-type (FR↔RL or FL↔RR)",
                "current": "0=FR(CCW), 1=RL(CCW), 2=FL(CW), 3=RR(CW)",
                "suggest": "Check if FR and RL (or FL and RR) are physically or logically swapped",
                "reason":  (
                    f"Pitch-yaw coupling r={py_corr:+.3f}: when nose pitches up/down, "
                    "the aircraft also yaws.  This is the signature of swapping two "
                    "motors that share the same spin direction but are on opposite ends "
                    "of the aircraft (diagonal same-type swap).  "
                    "Verify X-Plane motor index assignments and PWM_MAIN_FUNC values."
                ),
                "priority": "HIGH",
            })

    # ── T4: Heading convergence ───────────────────────────────────────────────
    if len(df_mission) >= MIN_N * 2 and "nav_bearing_deg" in df_mission.columns:
        df_t4 = df_mission[df_mission["nav_bearing_deg"] != 0].copy()
        if len(df_t4) >= MIN_N:
            df_t4["abs_err"] = df_t4.apply(
                lambda r: abs(heading_error(r["heading_deg"], r["nav_bearing_deg"])), axis=1)
            half = len(df_t4) // 2
            err_first  = df_t4["abs_err"].iloc[:half].mean()
            err_second = df_t4["abs_err"].iloc[half:].mean()
            if err_first > 10 and err_second > err_first * 1.30:
                findings.append((
                    "HIGH",
                    f"HEADING ERROR GROWING: |error| increased from "
                    f"{err_first:.1f}° to {err_second:.1f}° over the mission "
                    f"(+{(err_second/err_first-1)*100:.0f}%).  "
                    "Yaw corrections are making things worse, not better — "
                    "consistent with reversed yaw authority."
                ))

    return findings, recs


# ── main ───────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Analyze px4xplane flight log for yaw/turning issues")
    parser.add_argument("csv", nargs="?",
                        help="CSV recorded by record_flight.py (default: newest flightlog_*.csv)")
    parser.add_argument("--plot", action="store_true",
                        help="Show matplotlib time-series plots")
    parser.add_argument("--airframe", default="ehang184",
                        help="Airframe name for specific recommendations (default: ehang184)")
    args = parser.parse_args()

    # ── find CSV ──────────────────────────────────────────────────────────────
    if args.csv:
        csv_path = args.csv
    else:
        candidates = sorted(
            [f for f in os.listdir(".") if f.startswith("flightlog_") and f.endswith(".csv")],
            reverse=True)
        if not candidates:
            # try tools dir
            candidates = sorted(
                [os.path.join("tools", f) for f in os.listdir("tools")
                 if f.startswith("flightlog_") and f.endswith(".csv")],
                reverse=True) if os.path.isdir("tools") else []
        if not candidates:
            print("ERROR: no flightlog_*.csv found.  Pass a filename explicitly.")
            sys.exit(1)
        csv_path = candidates[0]
        print(f"Using newest log: {csv_path}")

    # ── load ──────────────────────────────────────────────────────────────────
    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        print(f"ERROR loading {csv_path}: {e}")
        sys.exit(1)

    print(f"\n{'='*70}")
    print(f"  FLIGHT LOG ANALYSIS: {os.path.basename(csv_path)}")
    print(f"{'='*70}")
    print(f"  Rows:        {len(df)}")
    print(f"  Duration:    {df['wall_time_s'].max():.1f} s")
    avg_dt = df["wall_time_s"].diff().median()
    print(f"  Avg rate:    {1/avg_dt:.1f} Hz" if avg_dt and avg_dt > 0 else "  Avg rate:    unknown")

    # ── filter segments ───────────────────────────────────────────────────────
    df_armed   = df[df["armed"] == 1].copy()
    df_mission = df[
        (df["armed"] == 1) &
        (df["main_mode"] == MAIN_MODE_AUTO) &
        (df["sub_mode"] == SUB_MODE_AUTO_MISSION)
    ].copy()
    df_hover = df[
        (df["armed"] == 1) &
        (df["main_mode"].isin([2, 3, 4])) &  # ALTCTL / POSCTL / AUTO
        (df["groundspeed_ms"] < 1.0)           # near stationary
    ].copy()

    print(f"\n  Armed rows:          {len(df_armed)}")
    print(f"  Mission mode rows:   {len(df_mission)}")
    print(f"  Near-hover rows:     {len(df_hover)}")

    # ── yaw statistics ────────────────────────────────────────────────────────
    print(f"\n{'─'*70}")
    print("  YAW RATE STATISTICS")
    print(f"{'─'*70}")
    print(describe(df["yawrate_rads"],          "All rows           yaw rate",  "rad/s"))
    if len(df_armed) > 0:
        print(describe(df_armed["yawrate_rads"], "Armed              yaw rate",  "rad/s"))
    if len(df_mission) > 0:
        print(describe(df_mission["yawrate_rads"],"Mission mode       yaw rate",  "rad/s"))
    if len(df_hover) > 0:
        print(describe(df_hover["yawrate_rads"],  "Near-hover         yaw rate",  "rad/s"))

    print(f"\n  Fraction of time |yaw_rate| > {math.degrees(SPIN_THRESHOLD):.0f} deg/s:")
    for label, dff in [("all", df), ("armed", df_armed), ("mission", df_mission)]:
        if len(dff) == 0:
            continue
        frac = (dff["yawrate_rads"].abs() > SPIN_THRESHOLD).mean()
        dominant = "CCW" if dff["yawrate_rads"].mean() < 0 else "CW"
        print(f"    {label:10s}: {frac*100:.1f}%  (dominant direction: {dominant})")

    # ── heading error ─────────────────────────────────────────────────────────
    if len(df_mission) > 10 and "nav_bearing_deg" in df_mission.columns:
        print(f"\n{'─'*70}")
        print("  HEADING vs NAV BEARING (mission mode)")
        print(f"{'─'*70}")
        df_nav = df_mission[df_mission["nav_bearing_deg"] != 0].copy()
        if len(df_nav) > 5:
            df_nav["hdg_err_deg"] = df_nav.apply(
                lambda r: heading_error(r["heading_deg"], r["nav_bearing_deg"]), axis=1)
            print(describe(df_nav["hdg_err_deg"], "heading vs nav_bearing error", "deg"))
            large_err = (df_nav["hdg_err_deg"].abs() > HEADING_ERROR_THRESHOLD_DEG).mean()
            print(f"  Time with |error| > {HEADING_ERROR_THRESHOLD_DEG:.0f} deg: {large_err*100:.1f}%")

    # ── attitude angles ───────────────────────────────────────────────────────
    if len(df_mission) > 0:
        print(f"\n{'─'*70}")
        print("  ATTITUDE DURING MISSION")
        print(f"{'─'*70}")
        print(describe(df_mission["roll_deg"],  "roll",  "deg"))
        print(describe(df_mission["pitch_deg"], "pitch", "deg"))
        print(describe(df_mission["yaw_deg"],   "yaw",   "deg"))

    # ── tumble / instability detection ───────────────────────────────────────
    if len(df_mission) > 0:
        roll_range = df_mission["roll_deg"].max() - df_mission["roll_deg"].min()
        roll_std   = df_mission["roll_deg"].std()
        pitch_std  = df_mission["pitch_deg"].std()
        spin_cls   = classify_spin(
            df_mission["yawrate_rads"].mean() if len(df_mission) > 0 else 0.0)
        if roll_range > 120 or roll_std > 35:
            print(f"\n{'█'*70}")
            print("  !! CATASTROPHIC INSTABILITY DETECTED !!")
            print(f"{'█'*70}")
            print(f"  Roll range : {roll_range:.0f} deg  (std {roll_std:.1f} deg)")
            print(f"  Pitch std  : {pitch_std:.1f} deg")
            print(f"  Yaw class  : {spin_cls}")
            print()
            print("  The aircraft is TUMBLING — this is not a tuning issue.  Root cause:")
            print("  1. Most likely: all CA_ROTORx_KM signs are inverted → positive-")
            print("     feedback runaway → cascade roll/pitch instability.")
            print("  2. Less likely: X-Plane aircraft model has all props same direction.")
            print()
            print("  DO NOT fly until KM signs are verified (see recommendations below).")
            print(f"{'█'*70}")

    # ── motor mapping diagnostics ─────────────────────────────────────────────
    mm_findings, mm_recs = detect_motor_mapping(df_armed, df_mission)

    print(f"\n{'─'*70}")
    print("  MOTOR MAPPING DIAGNOSTICS")
    print(f"{'─'*70}")
    print("  (inferred from attitude dynamics — no motor output telemetry needed)")
    print()

    SEVERITY_ICON = {"CRITICAL": "✖ CRITICAL", "HIGH": "⚠ HIGH", "WARNING": "• WARNING"}
    if not mm_findings:
        print("  No motor mapping anomalies detected.")
    else:
        for sev, msg in mm_findings:
            icon = SEVERITY_ICON.get(sev, sev)
            # Word-wrap
            words = f"[{icon}] {msg}".split()
            line = "  "
            for word in words:
                if len(line) + len(word) > 76:
                    print(line)
                    line = "    " + word + " "
                else:
                    line += word + " "
            if line.strip():
                print(line)
            print()

    # ── ehang184 known params (from airframe file) ─────────────────────────────
    CURRENT_PARAMS = {
        "MC_YAWRATE_P":   2.0,
        "MC_YAWRATE_I":   0.1,
        "MC_YAWRATE_D":   0.0,
        "MC_YAWRATE_FF":  0.2,
        "MC_YAWRATE_K":   0.9,
        "MC_YAWRATE_MAX": 200.0,
        "MC_YAW_P":       0.6,
        "MC_YAW_WEIGHT":  0.4,
        "MIS_YAW_ERR":    12.0,
        "IMU_GYRO_RATEMAX": 400,
        "CA_ROTOR0_KM":  -0.05,
        "CA_ROTOR1_KM":  -0.05,
        "CA_ROTOR2_KM":   0.05,
        "CA_ROTOR3_KM":   0.05,
    }

    # ── recommendations ────────────────────────────────────────────────────────
    warnings, recs = recommend(df_mission, df_hover, CURRENT_PARAMS, args.airframe)
    # Prepend motor mapping recs (they are typically higher priority)
    recs = mm_recs + recs

    print(f"\n{'═'*70}")
    print("  ROOT CAUSE ANALYSIS")
    print(f"{'═'*70}")

    if not warnings:
        print("  No significant turning/yaw drift detected.")
    else:
        for w in warnings:
            print(f"  ⚠  {w}")

    print(f"\n{'─'*70}")
    print("  PARAMETER RECOMMENDATIONS")
    print(f"{'─'*70}")

    if not recs:
        print("  No parameter changes recommended.")
    else:
        # Sort by priority: CRITICAL > HIGH > MEDIUM > LOW
        order = {"CRITICAL": 0, "HIGH": 1, "MEDIUM": 2, "LOW": 3}
        recs.sort(key=lambda r: order.get(r["priority"], 9))

        for i, r in enumerate(recs, 1):
            print(f"\n  [{i}] [{r['priority']}] {r['param']}")
            if r.get("current") is not None:
                print(f"       Current → {r['current']}")
            if r.get("suggest") is not None:
                print(f"       Suggest → {r['suggest']}")
            # Word-wrap reason
            words = r["reason"].split()
            line = "       Why: "
            for word in words:
                if len(line) + len(word) > 78:
                    print(line)
                    line = "             " + word + " "
                else:
                    line += word + " "
            if line.strip():
                print(line)

    print(f"\n{'─'*70}")
    print("  PX4 PARAM COMMANDS (copy-paste to NSH / MAVLink console)")
    print(f"{'─'*70}")
    for r in recs:
        param = r["param"].split("/")[0].strip()
        suggest = r.get("suggest")
        if isinstance(suggest, (int, float)):
            print(f"  param set {param} {suggest}")

    # ── directional quick-fix ──────────────────────────────────────────────────
    mean_yr   = df["yawrate_rads"].mean() if len(df) > 0 else 0
    spin_cls  = classify_spin(mean_yr)
    direction = "CCW" if mean_yr < 0 else "CW"
    if abs(mean_yr) > SPIN_THRESHOLD:
        print(f"\n{'─'*70}")
        if spin_cls == "RUNAWAY":
            print(f"  {direction} RUNAWAY QUICK-FIX (apply in order, retest after each)")
            print(f"{'─'*70}")
            print(f"  *** RUNAWAY at {abs(math.degrees(mean_yr)):.0f} deg/s — "
                  "controller is amplifying the spin ***")
            print()
            print("  STEP 1 — Test KM sign inversion (most likely root cause):")
            print("    param set CA_ROTOR0_KM -0.05   # was +0.05 (FR, CCW)")
            print("    param set CA_ROTOR1_KM -0.05   # was +0.05 (RL, CCW)")
            print("    param set CA_ROTOR2_KM  0.05   # was -0.05 (FL, CW)")
            print("    param set CA_ROTOR3_KM  0.05   # was -0.05 (RR, CW)")
            print("    → If spin direction reverses: KM signs confirmed wrong.")
            print("      Also update the X-Plane prop spin directions to match.")
            print()
            print("  STEP 2 — Raise rate limit so controller can arrest recovery spin:")
            print("    param set MC_YAWRATE_MAX 200.0")
            print()
            print("  STEP 3 — After fixing signs, add integral for residual bias:")
            print("    param set MC_YAWRATE_I 0.1")
            print()
            print("  STEP 4 — If spin persists (wrong magnitude, not wrong sign):")
            print("    Increase |KM| to 0.08 on all four rotors")
            print("    (more yaw authority relative to thrust)")
        else:
            print(f"  {direction} DRIFT QUICK-FIX (apply in order, test after each)")
            print(f"{'─'*70}")
            print("  1.  param set MC_YAWRATE_I 0.1    # integral — most likely fix")
            print("  2.  param set MC_YAWRATE_K 0.9    # increase P gain")
            print("  3.  param set MIS_YAW_ERR 12.0    # tighter yaw gate")
            print("  4.  Verify X-Plane motor spin directions match KM signs:")
            print("       Motor 0 (FR): CCW  Motor 1 (RL): CCW  KM=+0.05")
            print("       Motor 2 (FL): CW   Motor 3 (RR): CW   KM=−0.05")
            print("  5.  If still drifting: CA_ROTOR2_KM = CA_ROTOR3_KM = −0.04")

    print(f"\n{'='*70}\n")

    # ── optional plots ─────────────────────────────────────────────────────────
    if args.plot:
        try:
            import matplotlib.pyplot as plt
            import matplotlib.patches as mpatches

            fig = plt.figure(figsize=(16, 12))
            fig.suptitle(f"Flight Analysis: {os.path.basename(csv_path)}", fontsize=12)
            # Left column: time-series (4 rows); right column: coupling scatter (2 rows)
            gs = fig.add_gridspec(4, 2, width_ratios=[3, 1], hspace=0.35, wspace=0.3)
            axes = [fig.add_subplot(gs[i, 0]) for i in range(4)]
            for i in range(1, 4):
                axes[i].sharex(axes[0])
            t = df["wall_time_s"]

            # Mission mode shading helper
            def shade_mission(ax):
                in_mission = False
                t0 = None
                for _, row in df.iterrows():
                    is_m = (row["main_mode"] == MAIN_MODE_AUTO and
                            row["sub_mode"] == SUB_MODE_AUTO_MISSION)
                    if is_m and not in_mission:
                        t0 = row["wall_time_s"]
                        in_mission = True
                    elif not is_m and in_mission:
                        ax.axvspan(t0, row["wall_time_s"], alpha=0.12,
                                   color="green", label="Mission mode")
                        in_mission = False
                if in_mission:
                    ax.axvspan(t0, df["wall_time_s"].iloc[-1], alpha=0.12,
                               color="green")

            # 1. Yaw rate
            ax = axes[0]
            ax.plot(t, np.degrees(df["yawrate_rads"]), lw=0.8, label="yaw rate")
            ax.axhline(0, color="k", lw=0.5, ls="--")
            ax.axhline( math.degrees(SPIN_THRESHOLD), color="r", lw=0.5, ls=":")
            ax.axhline(-math.degrees(SPIN_THRESHOLD), color="r", lw=0.5, ls=":")
            shade_mission(ax)
            ax.set_ylabel("Yaw rate (deg/s)")
            ax.legend(fontsize=8)
            ax.grid(True, alpha=0.3)

            # 2. Heading vs nav_bearing
            ax = axes[1]
            ax.plot(t, df["heading_deg"], lw=0.8, label="heading", color="blue")
            if "nav_bearing_deg" in df.columns:
                mask = df["nav_bearing_deg"] != 0
                ax.plot(t[mask], df["nav_bearing_deg"][mask], lw=0.8,
                        label="nav_bearing", color="orange", ls="--")
            shade_mission(ax)
            ax.set_ylabel("Heading / bearing (deg)")
            ax.legend(fontsize=8)
            ax.grid(True, alpha=0.3)

            # 3. Roll and pitch
            ax = axes[2]
            ax.plot(t, df["roll_deg"],  lw=0.8, label="roll")
            ax.plot(t, df["pitch_deg"], lw=0.8, label="pitch")
            shade_mission(ax)
            ax.set_ylabel("Roll/Pitch (deg)")
            ax.legend(fontsize=8)
            ax.grid(True, alpha=0.3)

            # 4. Altitude
            ax = axes[3]
            ax.plot(t, df["alt_m"], lw=0.8, label="alt (m)")
            shade_mission(ax)
            ax.set_ylabel("Altitude (m)")
            ax.set_xlabel("Time (s)")
            ax.legend(fontsize=8)
            ax.grid(True, alpha=0.3)

            # Right column: roll-yaw and pitch-yaw scatter
            ax_ry = fig.add_subplot(gs[0:2, 1])
            ax_py = fig.add_subplot(gs[2:4, 1])

            ref_plot = df_armed if len(df_armed) > 10 else df
            ry_c = float(ref_plot["roll_deg"].corr(ref_plot["yawrate_rads"]))
            py_c = float(ref_plot["pitch_deg"].corr(ref_plot["yawrate_rads"]))

            ax_ry.scatter(ref_plot["roll_deg"],
                          np.degrees(ref_plot["yawrate_rads"]),
                          s=2, alpha=0.3, c="steelblue")
            ax_ry.set_xlabel("Roll (deg)", fontsize=8)
            ax_ry.set_ylabel("Yaw rate (deg/s)", fontsize=8)
            color_ry = "red" if abs(ry_c) > 0.45 else ("orange" if abs(ry_c) > 0.30 else "green")
            ax_ry.set_title(f"Roll-Yaw  r={ry_c:+.3f}", fontsize=9, color=color_ry)
            ax_ry.axhline(0, color="k", lw=0.5, ls="--")
            ax_ry.axvline(0, color="k", lw=0.5, ls="--")
            ax_ry.grid(True, alpha=0.3)

            ax_py.scatter(ref_plot["pitch_deg"],
                          np.degrees(ref_plot["yawrate_rads"]),
                          s=2, alpha=0.3, c="darkorange")
            ax_py.set_xlabel("Pitch (deg)", fontsize=8)
            ax_py.set_ylabel("Yaw rate (deg/s)", fontsize=8)
            color_py = "red" if abs(py_c) > 0.40 else ("orange" if abs(py_c) > 0.25 else "green")
            ax_py.set_title(f"Pitch-Yaw  r={py_c:+.3f}", fontsize=9, color=color_py)
            ax_py.axhline(0, color="k", lw=0.5, ls="--")
            ax_py.axvline(0, color="k", lw=0.5, ls="--")
            ax_py.grid(True, alpha=0.3)

            mission_patch = mpatches.Patch(color="green", alpha=0.3, label="Mission mode")
            fig.legend(handles=[mission_patch], loc="upper right", fontsize=8)
            plt.show()

        except ImportError:
            print("matplotlib not available — install with: pip install matplotlib")


if __name__ == "__main__":
    main()
