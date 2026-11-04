"""Understeer / oversteer test suite for the planar car + rear TV.

    python -m examples.balance            # full suite (~150 runs, parallel)
    python -m examples.balance --quick    # 3 TV variants, one speed, short bisection

Understeer (front axle saturates):
  U1  constant radius 9.125 m, speed ramp, TV off      -> K_us(ay), handling diagram, which axle saturates
  U2  fixed steer L/R, speed ramp, every TV variant    -> how much speed TV buys before the push
  U3  constant speed 8/10/12 m/s, steer ramp 1 deg/s   -> yaw gain vs the neutral line, K_us by speed
Corner entry:
  E1  straight, then a 0.1 s steer step to 50% / 90% of the limit -> overshoot, settling, TV lag
Oversteer (rear axle saturates):
  O1  U1's ramp with TV on                              -> does TV over-rotate the car into a spin?
  O2  steady circle at 90% of the limit + one event     -> recovery: yaw-rate kick (+0.3 rad/s), rear grip drop,
                                                           rear torque pulse, lift-off
  O3  bisection on event size                           -> spin margin per TV variant

TV variants: off, inside_cut, inside_cut_cap (r_ref capped at 0.95*ay_lim/u), oracle (uncapped).
Left turns only (tests/test_skidpad_runner.py shows right is the mirror image).
Plots, summary.csv and kus_table.csv go to outputs/balance/.
"""
import argparse
import csv
import math
import os
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from Vehicle import create_skidpad
from Vehicle.VCU.Configs.reartv1 import reartv1
from Vehicle.Simulation import balance as sim
from Data import balanceAnalysis as an
from Data import skidpadAnalysis as skid
from types import SimpleNamespace

R = sim.SKIDPAD_RADIUS_M
T_EVENT = 3.0
O2_RUN_S = 8.0
E1_STEP_S = 1.0
E1_RAMP_S = 0.1
U3_RATE_DEG_S = 1.0
U3_MAX_DEG = 25.0
EVENT_DEFAULT = {"yaw_kick": 0.3, "rear_grip": 0.2, "rear_torque": 100.0, "lift_off": 0.5}
EVENT_BOUNDS = {"rear_grip": (0.0, 0.7), "rear_torque": (0.0, 248.0)}


def eventDuration(event, severity):
    return {"yaw_kick": 0.01, "rear_grip": 1.0, "rear_torque": 0.5}.get(event, severity)


# ---------------------------------------------------------------- one run (picklable spec -> result)

def makeTV(variant, ay_cap):
    mode = "inside_cut" if variant == "inside_cut_cap" else variant
    return sim.CappedRearTV.fromConfig(reartv1, mode,
                                       ay_max_mps2=ay_cap if variant == "inside_cut_cap" else None)


def makeDisturbance(car, event, severity):
    if event == "yaw_kick":
        return sim.yawRateKick(T_EVENT, severity)
    if event == "rear_grip":
        return sim.rearGripDrop(car, T_EVENT, 1.0 - severity, 1.0)
    if event == "rear_torque":
        return sim.rearTorquePulse(T_EVENT, severity, 0.5)
    if event == "lift_off":
        return sim.liftOff(T_EVENT, severity)
    raise ValueError(event)


def runJob(spec):
    car = create_skidpad()
    L = car.wheelbase
    tv = makeTV(spec["variant"], spec.get("ay_cap"))
    kind = spec["kind"]

    if kind == "cr_ramp":                                   # U1 / O1
        driver = sim.circleDriver(car, lambda t: 6.0 + 0.25 * t)
        x0, c = car.getStartStateOnCircle(6.0, R, "left")
        return sim.runManeuver(car, driver, tv, 40.0, x0, c, R)

    if kind == "fixed_ramp":                                # U2
        steer = L / R
        driver = sim.fixedDriver(car, lambda t: 5.0 + 0.4 * t, lambda t: steer)
        x0, c = car.getStartStateOnCircle(5.0, R, "left")
        return sim.runManeuver(car, driver, tv, 25.0, x0, c, R)

    if kind == "steer_ramp":                                # U3
        u = spec["speed"]
        d0 = math.radians(1.0)
        rate = math.radians(U3_RATE_DEG_S)
        top = math.radians(U3_MAX_DEG)
        driver = sim.fixedDriver(car, lambda t: u, lambda t: min(d0 + rate * t, top))
        x0, _ = car.getStartStateOnCircle(u, L / math.tan(d0), "left")
        return sim.runManeuver(car, driver, tv, (U3_MAX_DEG - 1.0) / U3_RATE_DEG_S, x0,
                               stop_on_radius_error=False)

    if kind == "step":                                      # E1
        u, d_step = spec["speed"], spec["delta_step"]

        def steer(t):
            if t < E1_STEP_S - 1e-9:
                return 0.0
            return d_step * min(1.0, (t - E1_STEP_S + 0.01) / E1_RAMP_S)

        driver = sim.fixedDriver(car, lambda t: u, steer)
        return sim.runManeuver(car, driver, tv, 4.0, sim.straightStartState(car, u),
                               stop_on_radius_error=False)

    if kind == "disturb":                                   # O2 / O3
        u = spec["u_op"]
        if spec["driver"] == "fixed":
            d_ss = spec["delta_ss"]
            driver = sim.fixedDriver(car, lambda t: u, lambda t: d_ss)
        else:
            driver = sim.circleDriver(car, lambda t: u)
        x0, c = car.getStartStateOnCircle(u, R, "left")
        dist = makeDisturbance(car, spec["event"], spec["severity"])
        return sim.runManeuver(car, driver, tv, O2_RUN_S, x0, c, R, disturbance=dist,
                               stop_on_radius_error=spec["driver"] == "circle")

    if kind == "calibrate":                                 # O2 event size: TV off must recover
        severity = spec["severity"]
        for attempt in range(4):
            r = runJob(dict(spec, kind="disturb", severity=severity))
            if an.recovered(r, T_EVENT):
                return {"event": spec["event"], "severity": severity, "softened": attempt}
            severity /= 2          # every event's size is 'bigger = worse', so halving softens it
        return {"event": spec["event"], "severity": severity, "softened": 4}

    if kind == "bisect":                                    # O3
        lo, hi = EVENT_BOUNDS[spec["event"]]
        history = []

        def ok(sev):
            r = runJob(dict(spec, kind="disturb", severity=sev))
            good = an.recovered(r, T_EVENT)
            history.append((sev, good, r["stop_reason"]))
            return good

        if ok(hi):
            return {"lo": hi, "hi": hi, "never_failed": True, "history": history}
        for _ in range(spec["steps"]):
            mid = (lo + hi) / 2
            if ok(mid):
                lo = mid
            else:
                hi = mid
        return {"lo": lo, "hi": hi, "never_failed": False, "history": history}

    raise ValueError(kind)


def runAll(specs, workers):
    if workers <= 1:
        return [runJob(s) for s in specs]
    with ProcessPoolExecutor(max_workers=workers) as pool:
        return list(pool.map(runJob, specs))


# ---------------------------------------------------------------- helpers

def deltaAt(result, channel, value, start_s=1.0, half_window=10):
    """Median steer around the first tick where `channel` reaches `value`."""
    idx = np.flatnonzero((result["time_s"] >= start_s) & (result[channel] >= value))
    if not len(idx):
        return math.nan
    i = idx[0]
    return float(np.median(result["delta_rad"][max(0, i - half_window): i + half_window + 1]))


def writeCsv(rows, path):
    keys = []
    for row in rows:
        for k in row:
            if k not in keys:
                keys.append(k)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for row in rows:
            w.writerow({k: (f"{v:.5g}" if isinstance(v, float) else v) for k, v in row.items()})


def fmt(x, spec=".2f"):
    return "—" if x is None or (isinstance(x, float) and not math.isfinite(x)) else format(x, spec)


# ---------------------------------------------------------------- main

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--quick", action="store_true")
    parser.add_argument("--workers", type=int, default=min(5, max(1, (os.cpu_count() or 2) - 3)))
    args = parser.parse_args()

    variants = ["off", "inside_cut", "inside_cut_cap"] if args.quick else list(an.VARIANTS)
    tv_variants = [v for v in variants if v != "off"]
    speeds = [10.0] if args.quick else [8.0, 10.0, 12.0]
    bisect_steps = 4 if args.quick else 7
    out = Path(__file__).resolve().parents[1] / "outputs" / "balance"
    out.mkdir(parents=True, exist_ok=True)
    car = create_skidpad()
    L = car.wheelbase
    summary = []
    clock = time.time()

    # ---- U1: plant characterization (everything else is sized from it)
    u1 = runJob({"kind": "cr_ramp", "variant": "off"})
    ay_lim = an.limitAy(u1)
    K_lin = an.kusLinear(u1, L, 2.0, 0.6 * ay_lim)
    first = an.firstSaturatingAxle(u1)
    u_op = math.sqrt(0.9 * ay_lim * R)
    window = (u1["u_mps"] > u_op - 0.25) & (u1["u_mps"] < u_op + 0.25) & (u1["time_s"] > 1.0)
    delta_ss = float(np.median(u1["delta_rad"][window]))
    ay_cap = 0.95 * ay_lim
    r_ref_shift = K_lin * u_op ** 2 / L
    print(f"[U1] {time.time() - clock:5.1f}s  ay_lim {ay_lim:.2f} m/s² ({ay_lim / an.G:.2f} g), "
          f"K_us linear {an.kusDegPerG(K_lin):+.3f} deg/g, first to saturate: {first[0]} at {first[1] / an.G:.2f} g, "
          f"stop: {u1['stop_reason']} at {u1['time_s'][-1]:.1f}s")
    print(f"     u_op {u_op:.2f} m/s, δ_ss {math.degrees(delta_ss):.2f}°, cap ay_max {ay_cap:.2f} m/s², "
          f"r_ref shift from K_us at u_op {100 * r_ref_shift:+.2f}% "
          f"({'rerun with measured K_us' if abs(r_ref_shift) > 0.02 else 'K_us=0 is fine'})")

    c, med = an.kusLocal(u1, L)
    cu, medu = an.binnedMedian(u1["ay_mps2"], u1["u_mps"])
    with open(out / "kus_table.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["ay_mps2", "u_mps", "K_us_rad_per_mps2", "K_us_deg_per_g"])
        for a, k in zip(c, med):
            u_here = np.interp(a, cu, medu) if len(cu) else math.nan
            w.writerow([f"{a:.3f}", f"{u_here:.3f}", f"{k:.6g}", f"{an.kusDegPerG(k):.4f}"])

    summary.append({"test": "U1", "variant": "off", "stop": u1["stop_reason"], "t_end_s": u1["time_s"][-1],
                    "ay_lim_g": ay_lim / an.G, "K_us_linear_deg_per_g": an.kusDegPerG(K_lin),
                    "first_saturating_axle": first[0], "first_saturating_ay_g": first[1] / an.G,
                    "u_op_mps": u_op, "delta_ss_deg": math.degrees(delta_ss)})

    # ---- U2 + O1 + U3 in one parallel batch
    specs = [{"kind": "fixed_ramp", "variant": v, "ay_cap": ay_cap} for v in variants]
    specs += [{"kind": "cr_ramp", "variant": v, "ay_cap": ay_cap} for v in tv_variants]
    specs += [{"kind": "steer_ramp", "variant": v, "ay_cap": ay_cap, "speed": u} for u in speeds for v in variants]
    results = runAll(specs, args.workers)
    u2, o1, u3 = {}, {"off": u1}, {}
    for spec, r in zip(specs, results):
        if spec["kind"] == "fixed_ramp":
            u2[spec["variant"]] = r
        elif spec["kind"] == "cr_ramp":
            o1[spec["variant"]] = r
        else:
            u3.setdefault(spec["speed"], {})[spec["variant"]] = r
    print(f"[U2/O1/U3] {time.time() - clock:5.1f}s  {len(specs)} runs")

    # ---- E1 steps (sized from U3 TV-off) and O2 event calibration
    e1_specs = []
    for u in speeds:
        for level in (0.5, 0.9):
            d_step = deltaAt(u3[u]["off"], "ay_mps2", level * ay_lim, start_s=0.5)
            for v in variants:
                e1_specs.append({"kind": "step", "variant": v, "ay_cap": ay_cap, "speed": u, "level": level,
                                 "delta_step": d_step})
    events = list(EVENT_DEFAULT)
    drivers = ("fixed", "circle")
    cal_specs = [{"kind": "calibrate", "variant": "off", "event": ev, "severity": EVENT_DEFAULT[ev],
                  "driver": drv, "u_op": u_op, "delta_ss": delta_ss} for ev in events for drv in drivers]
    batch = runAll(e1_specs + cal_specs, args.workers)
    e1 = {}
    for spec, r in zip(e1_specs, batch[:len(e1_specs)]):
        e1.setdefault((spec["speed"], spec["level"]), {})[spec["variant"]] = r
    severity = {(s["event"], s["driver"]): c["severity"] for s, c in zip(cal_specs, batch[len(e1_specs):])}
    softened = {(s["event"], s["driver"]): c["softened"] for s, c in zip(cal_specs, batch[len(e1_specs):])}
    print(f"[E1/cal] {time.time() - clock:5.1f}s  event sizes {severity} (halvings {softened})")

    # ---- O2 matrix + O3 bisection chains
    o2_specs = [{"kind": "disturb", "variant": v, "ay_cap": ay_cap, "event": ev, "severity": severity[(ev, drv)],
                 "driver": drv, "u_op": u_op, "delta_ss": delta_ss}
                for ev in events for drv in drivers for v in variants]
    o3_specs = [{"kind": "bisect", "variant": v, "ay_cap": ay_cap, "event": ev, "driver": "circle",
                 "u_op": u_op, "delta_ss": delta_ss, "steps": bisect_steps}
                for ev in EVENT_BOUNDS for v in variants]
    batch = runAll(o2_specs + o3_specs, args.workers)
    o2 = {}
    for spec, r in zip(o2_specs, batch[:len(o2_specs)]):
        o2.setdefault(spec["event"], {})[(spec["driver"], spec["variant"])] = r
    margins = {(s["event"], s["variant"]): m for s, m in zip(o3_specs, batch[len(o2_specs):])}
    print(f"[O2/O3] {time.time() - clock:5.1f}s  {len(o2_specs)} runs + {len(o3_specs)} bisection chains")

    # ---------------------------------------------------------------- metrics
    print(f"\n{'U2 fixed steer L/R, speed ramp':<40}{'onset u':>9}{'onset ay':>10}{'stop':>16}{'at u':>8}"
          f"{'RMS yaw err':>13}{'sat ay':>9}{'sat frac':>10}")
    for v, r in u2.items():
        u_on, ay_on = an.understeerOnset(r, R)
        ns = SimpleNamespace(time_s=r["time_s"][r["time_s"] >= 1], ay=r["ay_mps2"][r["time_s"] >= 1],
                             is_saturated=r["is_saturated"][r["time_s"] >= 1] & (r["base_torque_Nm"][r["time_s"] >= 1] > 1e-6))
        sat_ay = skid.getSaturation(ns) if v in ("inside_cut", "inside_cut_cap") else None
        row = {"test": "U2", "variant": v, "stop": r["stop_reason"], "t_end_s": r["time_s"][-1],
               "onset_u_mps": u_on, "onset_ay_g": ay_on / an.G, "u_end_mps": r["u_mps"][-1],
               "rms_yaw_err_radps": an.rmsYawError(r), "saturation_ay_g": (sat_ay / an.G) if sat_ay else math.nan,
               "saturated_frac": an.saturatedFraction(r)}
        summary.append(row)
        print(f"  {an.VARIANT_STYLE[v]['label']:<38}{fmt(u_on):>9}{fmt(ay_on / an.G):>9}g{r['stop_reason']:>16}"
              f"{fmt(r['u_mps'][-1]):>8}{fmt(row['rms_yaw_err_radps'], '.3f'):>13}"
              f"{fmt(row['saturation_ay_g']):>8}g{fmt(row['saturated_frac']):>10}")
    if "oracle" in u2 and "inside_cut" in u2:
        def ns(r):
            m = r["time_s"] >= 1
            order = np.argsort(r["ay_mps2"][m])
            return SimpleNamespace(ay=r["ay_mps2"][m][order], external_yaw_moment_Nm=r["external_yaw_moment_Nm"][m][order],
                                   tv_yaw_moment_Nm=r["tv_yaw_moment_Nm"][m][order])
        print(f"  moment shortfall oracle vs inside cut: {skid.getShortfall(ns(u2['oracle']), ns(u2['inside_cut'])):.1f} Nm")

    print(f"\n{'O1 constant radius ramp, TV on':<40}{'stop':>16}{'at t':>8}{'max ay held':>13}{'peak |β|':>10}"
          f"{'r_ref>feasible':>16}{'Δδ @0.9 lim':>13}")
    d_off = deltaAt(u1, "ay_mps2", 0.9 * ay_lim)
    for v, r in o1.items():
        m = r["time_s"] >= 1
        infeasible = float(np.mean(r["r_ref_raw_radps"][m] > ay_lim / r["u_mps"][m]))
        d_v = deltaAt(r, "ay_mps2", 0.9 * ay_lim)
        row = {"test": "O1", "variant": v, "stop": r["stop_reason"], "t_end_s": r["time_s"][-1],
               "ay_held_g": an.limitAy(r) / an.G, "peak_beta_deg": float(np.degrees(np.max(np.abs(r["sideslip_rad"][m])))),
               "frac_rref_infeasible": infeasible, "steer_saved_deg_at_0.9lim": math.degrees(d_off - d_v)}
        summary.append(row)
        print(f"  {an.VARIANT_STYLE[v]['label']:<38}{r['stop_reason']:>16}{r['time_s'][-1]:>7.1f}s"
              f"{fmt(row['ay_held_g']):>12}g{fmt(row['peak_beta_deg'], '.1f'):>9}°{100 * infeasible:>15.0f}%"
              f"{fmt(row['steer_saved_deg_at_0.9lim'], '+.2f'):>12}°")

    print(f"\n{'U3 constant speed, steer ramp':<40}{'onset δ':>9}{'onset ay':>10}{'max ay':>9}{'K_us lin':>11}{'stop':>16}{'at δ':>8}")
    for u in speeds:
        for v, r in u3[u].items():
            d_on, ay_on = an.yawGainOnset(r, L)
            ay_max = an.limitAy(r, start_s=0.5)
            k_lin = an.kusLinear(r, L, 2.0, 0.6 * ay_lim, start_s=0.5)
            row = {"test": "U3", "variant": v, "speed_mps": u, "stop": r["stop_reason"], "t_end_s": r["time_s"][-1],
                   "onset_delta_deg": d_on, "onset_ay_g": ay_on / an.G, "ay_max_g": ay_max / an.G,
                   "K_us_linear_deg_per_g": an.kusDegPerG(k_lin), "delta_end_deg": math.degrees(r["delta_rad"][-1])}
            summary.append(row)
            print(f"  {u:>4g} m/s {an.VARIANT_STYLE[v]['label']:<30}{fmt(d_on, '.1f'):>8}°{fmt(ay_on / an.G):>9}g"
                  f"{fmt(ay_max / an.G):>8}g{fmt(row['K_us_linear_deg_per_g'], '+.3f'):>11}{r['stop_reason']:>16}"
                  f"{row['delta_end_deg']:>7.1f}°")

    e1_table = []
    print(f"\n{'E1 step steer from straight':<40}{'δ step':>8}{'rise':>8}{'overshoot':>11}{'settle':>8}{'peak |β|':>10}"
          f"{'cut onset':>11}{'cut peak':>10}{'stop':>12}")
    for (u, level), runs in sorted(e1.items()):
        for v, r in runs.items():
            met = an.stepMetrics(r, E1_STEP_S)
            d_step = next(s["delta_step"] for s in e1_specs if s["speed"] == u and s["level"] == level)
            row = {"test": "E1", "variant": v, "speed": u, "level": level, "delta_step_deg": math.degrees(d_step), **met}
            e1_table.append(row)
            summary.append(row)
            print(f"  {u:>4g} m/s {level:.0%} {an.VARIANT_STYLE[v]['label']:<26}{math.degrees(d_step):>7.2f}°"
                  f"{fmt(met.get('rise_s'), '.3f'):>8}{fmt(met.get('overshoot_pct'), '.1f'):>10}%{fmt(met.get('settle_s'), '.2f'):>8}"
                  f"{fmt(met.get('beta_peak_deg'), '.2f'):>9}°{fmt(met.get('cut_onset_s'), '.3f'):>11}"
                  f"{fmt(met.get('cut_peak_Nm'), '.1f'):>10}{met['stop']:>12}")

    o2_table = []
    print(f"\n{'O2 disturbance at u_op':<40}{'size':>8}{'recovered':>11}{'settle':>8}{'yaw over':>10}{'peak Δβ':>9}"
          f"{'rear util':>11}{'cut@rearsat':>12}{'stop':>16}")
    for ev in events:
        for (drv, v), r in o2[ev].items():
            met = an.disturbanceMetrics(r, T_EVENT, eventDuration(ev, severity[(ev, drv)]))
            row = {"test": "O2", "event": ev, "severity": severity[(ev, drv)], "driver": drv, "variant": v, **met}
            o2_table.append(row)
            summary.append(row)
            print(f"  {ev:<12}{drv:<7}{an.VARIANT_STYLE[v]['label']:<22}{severity[(ev, drv)]:>7.3g}{str(met['recovered']):>11}"
                  f"{fmt(met['settle_s']):>8}{fmt(met['yaw_overshoot_radps'], '.3f'):>10}{fmt(met['peak_dbeta_deg']):>8}°"
                  f"{fmt(met['peak_rear_util']):>11}{str(met['cut_while_rear_saturated']):>12}{met['stop']:>16}")

    print(f"\n{'O3 spin margin (pure pursuit at u_op)':<40}{'recovers up to':>16}{'fails at':>10}")
    for (ev, v), m in margins.items():
        row = {"test": "O3", "event": ev, "variant": v, "recovers_up_to": m["lo"],
               "fails_at": math.nan if m["never_failed"] else m["hi"], "never_failed": m["never_failed"]}
        summary.append(row)
        print(f"  {ev:<12}{an.VARIANT_STYLE[v]['label']:<28}{m['lo']:>16.3g}{fmt(row['fails_at'], '.3g'):>10}")

    writeCsv(summary, out / "summary.csv")

    # ---------------------------------------------------------------- plots
    an.plotU1Steer(u1, L, R, K_lin, ay_lim, out / "u1_steer_vs_ay.png")
    an.plotU1Handling(u1, ay_lim, out / "u1_handling_diagram.png")
    an.plotU1Axle(u1, ay_lim, first, out / "u1_axle_utilization.png")
    an.plotU2Path(u2, R, out / "u2_path_radius.png")
    an.plotU2Yaw(u2, out / "u2_yaw_tracking.png")
    an.plotU3Yaw(u3, L, ay_lim, out / "u3_yaw_vs_steer.png")
    an.plotU3Kus(u3, u1, L, out / "u3_kus_by_speed.png")
    an.plotU3Paths(u3, out / "u3_paths.png")
    an.plotE1Step(e1, E1_STEP_S, out / "e1_step_response.png")
    an.plotE1Metrics(e1_table, variants, out / "e1_metrics.png")
    an.plotO1(o1, ay_lim, out / "o1_overrotation.png")
    an.plotO1Phase(o1, out / "o1_phase_portrait.png")
    for ev in events:
        an.plotO2Event(ev, o2[ev], T_EVENT, {d: eventDuration(ev, severity[(ev, d)]) for d in drivers},
                       {d: severity[(ev, d)] for d in drivers}, out / f"o2_{ev}.png")
    an.plotO2Summary(o2_table, variants, out / "o2_summary.png")
    an.plotO3(margins, variants, out / "o3_spin_margin.png")
    print(f"\nplots, summary.csv, kus_table.csv -> {out}   ({time.time() - clock:.0f}s total)")


if __name__ == "__main__":
    main()
