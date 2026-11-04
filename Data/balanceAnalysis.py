"""Metrics and plots for the understeer / oversteer maneuvers (examples/balance.py).

Every function takes the result dicts from Vehicle.Simulation.balance.runManeuver (one numpy
array per channel). No Vehicle imports: geometry constants are passed in.

Sign conventions (left turn): r > 0, delta > 0, alpha_f - alpha_r > 0 means understeer,
K_us > 0 means understeer. K_us is in rad/(m/s^2) internally, deg/g when shown.
"""
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

G = 9.81

# ---------------------------------------------------------------- style (reference palette, light)
SURFACE = "#fcfcfb"
TEXT = "#0b0b0b"
TEXT_2 = "#52514e"
GRID = "#e6e5e1"
REF = "#52514e"                               # reference lines: kinematic, limits, targets
VARIANTS = ["off", "inside_cut", "inside_cut_cap", "oracle"]
VARIANT_STYLE = {
    "off":            dict(color="#8f8e8a", ls="-",  label="TV off"),
    "inside_cut":     dict(color="#2a78d6", ls="-",  label="inside cut"),
    "inside_cut_cap": dict(color="#eb6834", ls=(0, (5, 2)), label="inside cut + r_ref cap"),   # dashed: shows overlap
    "oracle":         dict(color="#1baf7a", ls="-.", label="oracle (ceiling)"),
}
AXLE_STYLE = {"front": dict(color="#6250d6", ls="-", label="front axle"),
              "rear": dict(color="#e34948", ls="-", label="rear axle")}
SPEED_RAMP = ["#86b6ef", "#2a78d6", "#104281"]   # ordinal blue: 8 / 10 / 12 m/s
SEQ_BLUE = ListedColormap(["#cde2fb", "#b7d3f6", "#9ec5f4", "#86b6ef", "#6da7ec", "#5598e7",
                           "#3987e5", "#2a78d6", "#256abf", "#1c5cab", "#184f95", "#104281"])
LW = 1.6

RC = {
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": GRID, "axes.labelcolor": TEXT_2, "axes.titlecolor": TEXT,
    "axes.titlesize": 10, "axes.titleweight": "bold", "axes.labelsize": 9,
    "xtick.color": TEXT_2, "ytick.color": TEXT_2, "xtick.labelsize": 8, "ytick.labelsize": 8,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
    "axes.spines.top": False, "axes.spines.right": False,
    "legend.fontsize": 8, "legend.frameon": False, "text.color": TEXT, "font.size": 9,
}


def _save(fig, path, title=None):
    if title:
        fig.suptitle(title, fontsize=11, fontweight="bold", color=TEXT)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def _line(ax, x, y, variant, **kw):
    style = dict(VARIANT_STYLE[variant])
    style.update(kw)
    return ax.plot(x, y, lw=LW, **style)


# ---------------------------------------------------------------- channel helpers

def axleUtil(result):
    util = result["tire_util"]
    return util[:, :2].mean(axis=1), util[:, 2:].mean(axis=1)


def axleSlip(result):
    sa = result["slip_angles_rad"]
    return (sa[:, 0] + sa[:, 1]) / 2, (sa[:, 2] + sa[:, 3]) / 2


def deg(x):
    return np.degrees(x)


def kusDegPerG(K):
    return np.degrees(K) * G


def binnedMedian(x, y, width=0.5, lo=None, hi=None, min_count=5):
    x, y = np.asarray(x), np.asarray(y)
    ok = np.isfinite(x) & np.isfinite(y)
    x, y = x[ok], y[ok]
    if len(x) == 0:
        return np.array([]), np.array([])
    lo = np.floor(x.min() / width) * width if lo is None else lo
    hi = np.ceil(x.max() / width) * width if hi is None else hi
    centres, medians = [], []
    for left in np.arange(lo, hi, width):
        m = (x >= left) & (x < left + width)
        if m.sum() >= min_count:
            centres.append(np.median(x[m]))          # where the samples are, not the bin midpoint
            medians.append(np.median(y[m]))
    return np.array(centres), np.array(medians)


def _binnedOn(x, y, keep, edges, min_count=5):
    """Median of y in each [edges[i], edges[i+1]) bin of x (NaN where too few samples): a common grid,
    so two runs can be subtracted bin by bin."""
    x, y = np.asarray(x)[keep], np.asarray(y)[keep]
    out = np.full(len(edges) - 1, np.nan)
    for i in range(len(edges) - 1):
        m = (x >= edges[i]) & (x < edges[i + 1])
        if m.sum() >= min_count:
            out[i] = np.median(y[m])
    return out


# ---------------------------------------------------------------- understeer metrics

def steerExcess(result, wheelbase_m):
    """Steer beyond the geometric angle for the path actually driven: delta - atan(L/R_path), R_path = u/r."""
    return result["delta_rad"] - np.arctan(wheelbase_m * result["r_radps"] / result["u_mps"])


def kusLinear(result, wheelbase_m, ay_lo, ay_hi, start_s=1.0):
    """SAE J266 understeer gradient: slope d(steer excess)/d(ay) over [ay_lo, ay_hi], rad/(m/s^2).
    A slope, not a ratio, so constant offsets (Ackermann, pure-pursuit cut) drop out."""
    ay = result["ay_mps2"]
    excess = steerExcess(result, wheelbase_m)
    m = (ay >= ay_lo) & (ay <= ay_hi) & (result["time_s"] >= start_s) & np.isfinite(excess)
    if m.sum() < 5:
        return math.nan
    return float(np.polyfit(ay[m], excess[m], 1)[0])


def kusLocal(result, wheelbase_m, width=0.5, start_s=1.0, min_ay=1.0):
    """Local understeer gradient K_us(ay): gradient of the binned-median steer excess vs ay."""
    ay = result["ay_mps2"]
    m = (result["time_s"] >= start_s) & (ay >= min_ay)
    c, med = binnedMedian(ay[m], steerExcess(result, wheelbase_m)[m], width)
    if len(c) < 3:
        return c, np.full(len(c), np.nan)
    return c, np.gradient(med, c)


def limitAy(result, hold_s=0.5, tol_m=0.5, start_s=1.0):
    """Highest ay held for hold_s with |radius error| < tol_m (no radius check if it's NaN)."""
    t, ay, rerr = result["time_s"], result["ay_mps2"], result["radius_error_m"]
    if len(t) < 3:
        return math.nan
    dt = t[1] - t[0]
    n = max(1, round(hold_s / dt))
    ok = (t >= start_s) & ((np.abs(rerr) < tol_m) | np.isnan(rerr))
    held = np.where(ok, ay, -np.inf)
    if len(held) < n:
        return math.nan
    windows = np.lib.stride_tricks.sliding_window_view(held, n)
    best = windows.min(axis=1).max()
    return float(best) if np.isfinite(best) else math.nan


def firstSaturatingAxle(result, threshold=0.98, start_s=1.0):
    """Which axle's mean combined grip use reaches `threshold` first: (axle, ay, time)."""
    t, ay = result["time_s"], result["ay_mps2"]
    front, rear = axleUtil(result)
    after = t >= start_s
    hits = {}
    for name, util in (("front", front), ("rear", rear)):
        idx = np.flatnonzero(after & (util >= threshold))
        if len(idx):
            hits[name] = idx[0]
    if not hits:
        return "neither", math.nan, math.nan
    name = min(hits, key=hits.get)
    return name, float(ay[hits[name]]), float(t[hits[name]])


def valueAtFirst(result, channel, mask_channel, threshold, start_s=1.0, window=5):
    """Median of `channel` around the first tick where mask_channel >= threshold."""
    t = result["time_s"]
    idx = np.flatnonzero((t >= start_s) & (result[mask_channel] >= threshold))
    if not len(idx):
        return math.nan
    i = idx[0]
    return float(np.median(result[channel][max(0, i - window): i + window + 1]))


def understeerOnset(result, radius_m, tol_m=0.5, start_s=1.0):
    """Fixed steer: first point where the path radius u/r is tol_m wider than radius_m -> (u, ay)."""
    t, u, r, ay = result["time_s"], result["u_mps"], result["r_radps"], result["ay_mps2"]
    r_path = u / np.maximum(r, 1e-3)
    idx = np.flatnonzero((t >= start_s) & (np.abs(r_path - radius_m) > tol_m))
    if not len(idx):
        return math.nan, math.nan
    return float(u[idx[0]]), float(ay[idx[0]])


def yawGainOnset(result, wheelbase_m, drop=0.05, start_s=1.0):
    """Constant speed steer ramp: first steer where r falls `drop` below the neutral u*tan(delta)/L."""
    t, u, r, delta, ay = (result[k] for k in ("time_s", "u_mps", "r_radps", "delta_rad", "ay_mps2"))
    neutral = u * np.tan(delta) / wheelbase_m
    idx = np.flatnonzero((t >= start_s) & (r < (1 - drop) * neutral) & (delta > math.radians(2)))
    if not len(idx):
        return math.nan, math.nan
    return float(np.degrees(delta[idx[0]])), float(ay[idx[0]])


def rmsYawError(result, start_s=1.0, ref="r_ref_raw_radps"):
    m = result["time_s"] >= start_s
    e = result["r_radps"][m] - result[ref][m]
    return float(np.sqrt(np.mean(e ** 2))) if len(e) else math.nan


def saturatedFraction(result, start_s=1.0):
    """Fraction of ticks with the cut at its clamp; ticks with zero base torque don't count
    (the parent controller flags those as saturated because cut 0 == base 0)."""
    m = (result["time_s"] >= start_s) & (result["base_torque_Nm"] > 1e-6)
    return float(np.mean(result["is_saturated"][m])) if m.any() else math.nan


# ---------------------------------------------------------------- transient metrics

def settlingTime(t, y, y_ss, band, t_from):
    """Time from t_from until |y - y_ss| stays inside band to the end. NaN if it never does."""
    after = t >= t_from - 1e-9
    outside = after & (np.abs(y - y_ss) > band)
    if not outside.any():
        return 0.0
    last = np.flatnonzero(outside)[-1]
    if last >= len(t) - 1:
        return math.nan
    return float(t[last + 1] - t_from)


def stepMetrics(result, t_step, final_window_s=0.5):
    """Yaw-rate step response after a steer step at t_step."""
    t, r, beta = result["time_s"], result["r_radps"], result["sideslip_rad"]
    out = {"stop": result["stop_reason"]}
    if result["stop_reason"] != "time_up":
        out.update(r_final=math.nan, rise_s=math.nan, peak_s=math.nan, overshoot_pct=math.nan,
                   settle_s=math.nan, beta_peak_deg=float(np.degrees(np.max(np.abs(beta)))))
        return out
    final = t >= t[-1] - final_window_s
    r_final = float(np.mean(r[final]))
    post = t >= t_step
    tp, rp = t[post], r[post]

    def firstReach(fraction):
        idx = np.flatnonzero(rp >= fraction * r_final)
        return float(tp[idx[0]]) if len(idx) else math.nan

    i_peak = int(np.argmax(rp))
    beta_ss = float(np.mean(beta[final]))
    out.update(
        r_final=r_final,
        rise_s=firstReach(0.9) - firstReach(0.1),
        peak_s=float(tp[i_peak] - t_step),
        overshoot_pct=float(100 * max(0.0, rp[i_peak] - r_final) / abs(r_final)),
        settle_s=settlingTime(t, r, r_final, 0.05 * abs(r_final), t_step),
        beta_peak_deg=float(np.degrees(np.max(np.abs(beta[post])))),
        beta_overshoot_deg=float(np.degrees(np.max(np.abs(beta[post] - beta_ss)))),
        cut_peak_Nm=float(np.max(result["inside_cut_Nm"][post])),
        yaw_err_rms_1s=float(np.sqrt(np.mean((result["r_radps"] - result["r_ref_filtered_radps"])
                                             [(t >= t_step) & (t < t_step + 1.0)] ** 2))),
    )
    cut_on = np.flatnonzero(post & (result["inside_cut_Nm"] > 1.0))
    out["cut_onset_s"] = float(t[cut_on[0]] - t_step) if len(cut_on) else math.nan
    return out


def baseline(result, t_event, window_s=0.5):
    t = result["time_s"]
    m = (t >= t_event - window_s) & (t < t_event)
    return float(np.mean(result["r_radps"][m])), float(np.mean(result["sideslip_rad"][m]))


def recovered(result, t_event, beta_tol_deg=2.0):
    """Ran to the end and sideslip came back within beta_tol_deg of where it was before the event."""
    if result["stop_reason"] != "time_up":
        return False
    _, beta_ss = baseline(result, t_event)
    return abs(math.degrees(result["sideslip_rad"][-1] - beta_ss)) < beta_tol_deg


def disturbanceMetrics(result, t_event, duration_s, band_radps=0.05):
    t = result["time_s"]
    r_ss, beta_ss = baseline(result, t_event)
    after = t >= t_event - 1e-9
    post_event = t >= t_event + duration_s - 1e-9
    rear = axleUtil(result)[1]
    ran = result["stop_reason"] == "time_up"
    rerr = result["radius_error_m"]
    worse = after & (result["inside_cut_Nm"] > 1e-6) & (rear >= 0.98)
    out = {
        "stop": result["stop_reason"],
        "recovered": recovered(result, t_event),
        "r_ss": r_ss,
        "beta_ss_deg": math.degrees(beta_ss),
        "settle_s": settlingTime(t, result["r_radps"], r_ss, band_radps, t_event) if ran else math.nan,
        "yaw_overshoot_radps": float(np.max(result["r_radps"][after] - r_ss)),
        "peak_dbeta_deg": float(np.degrees(np.max(np.abs(result["sideslip_rad"][after] - beta_ss)))),
        "peak_rear_util": float(np.max(rear[after])),
        "cut_while_rear_saturated": bool(worse.any()),   # cutting there frees rear grip: not automatically worse
        "cut_rear_sat_ticks": int(worse.sum()),
    }
    if np.isfinite(rerr).any():
        out["peak_radius_err_m"] = float(np.nanmax(np.abs(rerr[after])))
        out["post_radius_rmse_m"] = float(np.sqrt(np.nanmean(rerr[post_event] ** 2))) if post_event.any() else math.nan
    return out


# ---------------------------------------------------------------- plots: understeer

def plotU1Steer(u1, wheelbase_m, radius_m, K_lin, ay_lim, path):
    with plt.rc_context(RC):
        fig, (top, bottom) = plt.subplots(2, 1, figsize=(7.5, 6.5), sharex=True)
        m = u1["time_s"] >= 1.0
        ay_g = u1["ay_mps2"][m] / G
        top.plot(ay_g, deg(u1["delta_rad"][m]), lw=0.8, color=VARIANT_STYLE["off"]["color"], alpha=0.6)
        c, med = binnedMedian(u1["ay_mps2"][m], deg(u1["delta_rad"][m]))
        top.plot(c / G, med, "o-", ms=4, lw=LW, color=VARIANT_STYLE["off"]["color"], label="steer needed (TV off)")
        kin = math.degrees(math.atan(wheelbase_m / radius_m))
        top.axhline(kin, color=REF, ls="--", lw=1, label=f"geometric atan(L/R) = {kin:.2f}°")
        top.axvline(ay_lim / G, color=REF, ls=":", lw=1, label=f"limit {ay_lim / G:.2f} g")
        top.set_ylabel("road-wheel steer δ (deg)")
        top.set_title(f"Constant radius R = {radius_m} m, speed ramp — "
                      f"linear K_us = {kusDegPerG(K_lin):+.3f} deg/g")
        sub = c < 0.9 * ay_lim                # the plow spike sits in the last ~5% below the limit
        hi = np.nanmax(med[sub]) if sub.any() else kin + 1
        top.set_ylim(kin - 0.6, max(kin + 0.8, hi + 0.6))
        if len(med) and np.nanmax(med) > max(kin + 0.8, hi + 0.6):
            top.text(ay_lim / G, max(kin + 0.8, hi + 0.6), "front saturates:\nsteer runs off-scale  ",
                     ha="right", va="top", fontsize=8, color=TEXT_2)
        top.legend(loc="upper left")

        c, K = kusLocal(u1, wheelbase_m)
        med = kusDegPerG(K)
        bottom.plot(c / G, med, "o-", ms=4, lw=LW, color=VARIANT_STYLE["off"]["color"], label="K_us (TV off)")
        bottom.axhline(0, color=REF, lw=1)
        bottom.axvline(ay_lim / G, color=REF, ls=":", lw=1)
        bottom.text(0.01, 0.95, "above 0: understeer", transform=bottom.transAxes, va="top", color=TEXT_2, fontsize=8)
        bottom.text(0.01, 0.05, "below 0: oversteer", transform=bottom.transAxes, va="bottom", color=TEXT_2, fontsize=8)
        bottom.set_ylabel("understeer gradient K_us = dδ/d(ay) (deg/g)")
        bottom.set_xlabel("lateral acceleration (g)")
        sub = (c < 0.9 * ay_lim) & np.isfinite(med)
        lim = max(0.3, 1.4 * np.nanmax(np.abs(med[sub]))) if sub.any() else 1.0
        bottom.set_ylim(-lim, lim)
        if np.nanmax(np.abs(med)) > lim:
            bottom.text(ay_lim / G, lim, "plow-off: off-scale  ", ha="right", va="top", fontsize=8, color=TEXT_2)
        _save(fig, path)


def plotU1Handling(u1, ay_lim, path):
    with plt.rc_context(RC):
        fig, (left, right) = plt.subplots(1, 2, figsize=(10, 4.2), sharey=True)
        m = u1["time_s"] >= 1.0
        af, ar = axleSlip(u1)
        ay_g = u1["ay_mps2"][m] / G
        c, med = binnedMedian(u1["ay_mps2"][m], deg(af - ar)[m])
        left.plot(deg(af - ar)[m], ay_g, lw=0.8, color=VARIANT_STYLE["off"]["color"], alpha=0.5)
        left.plot(med, c / G, "o-", ms=4, lw=LW, color=VARIANT_STYLE["off"]["color"], label="TV off")
        left.axvline(0, color=REF, lw=1)
        left.axhline(ay_lim / G, color=REF, ls=":", lw=1, label=f"limit {ay_lim / G:.2f} g")
        left.set_xlim(-1.0, 1.5)
        left.set_xlabel("α_front − α_rear (deg)   ← oversteer | understeer →")
        left.set_ylabel("lateral acceleration (g)")
        left.set_title("Handling diagram")
        left.legend(loc="lower right")

        c, med = binnedMedian(u1["ay_mps2"][m], deg(u1["sideslip_rad"])[m])
        right.plot(deg(u1["sideslip_rad"])[m], ay_g, lw=0.8, color=VARIANT_STYLE["off"]["color"], alpha=0.5)
        right.plot(med, c / G, "o-", ms=4, lw=LW, color=VARIANT_STYLE["off"]["color"])
        right.axhline(ay_lim / G, color=REF, ls=":", lw=1)
        right.set_xlabel("body sideslip β (deg)")
        right.set_title("Sideslip budget")
        _save(fig, path)


def plotU1Axle(u1, ay_lim, first, path):
    with plt.rc_context(RC):
        fig, ax = plt.subplots(figsize=(7.5, 4.2))
        m = u1["time_s"] >= 1.0
        front, rear = axleUtil(u1)
        for name, util in (("front", front), ("rear", rear)):
            c, med = binnedMedian(u1["ay_mps2"][m], util[m], width=0.25)
            ax.plot(c / G, med, lw=LW, **AXLE_STYLE[name])
        ax.axhline(1.0, color=REF, ls="--", lw=1, label="at grip limit")
        ax.axvline(ay_lim / G, color=REF, ls=":", lw=1, label=f"limit {ay_lim / G:.2f} g")
        name, ay_first, _ = first
        if name != "neither":
            ax.annotate(f"{name} reaches 0.98 first ({ay_first / G:.2f} g)", xy=(ay_first / G, 0.98),
                        xytext=(0.45, 0.25), textcoords="axes fraction", color=TEXT, fontsize=8,
                        arrowprops=dict(arrowstyle="->", color=TEXT_2, lw=0.8))
        ax.set_xlabel("lateral acceleration (g)")
        ax.set_ylabel("combined grip use, axle mean (1 = limit)")
        ax.set_ylim(0, 1.1)
        ax.set_title("Which axle runs out first (constant radius, TV off)")
        ax.legend(loc="upper left")
        _save(fig, path)


def plotU2Path(u2, radius_m, path):
    with plt.rc_context(RC):
        fig, (left, right) = plt.subplots(1, 2, figsize=(11, 4.6))
        for v, r in u2.items():
            m = r["time_s"] >= 1.0
            _line(left, r["u_mps"][m], r["u_mps"][m] / np.maximum(r["r_radps"][m], 1e-3), v)
            _line(right, r["X_m"], r["Y_m"], v)
        left.axhline(radius_m, color=REF, ls="--", lw=1, label=f"skidpad R = {radius_m} m")
        left.axhline(radius_m + 0.5, color=REF, ls=":", lw=1, label="onset: +0.5 m")
        left.set_ylim(radius_m - 1.0, radius_m + 3.0)
        left.set_xlabel("speed (m/s)")
        left.set_ylabel("path radius u/r (m)")
        left.set_title("Fixed steer L/R, speed ramp: radius grows = understeer")
        left.legend(loc="upper left")
        th = np.linspace(0, 2 * math.pi, 300)
        right.plot(radius_m * np.cos(th), radius_m + radius_m * np.sin(th), color=REF, ls="--", lw=1)
        right.set_aspect("equal")
        right.set_xlabel("X (m)")
        right.set_ylabel("Y (m)")
        right.set_title("Trajectories (dashed: skidpad circle)")
        _save(fig, path)


def plotU2Yaw(u2, path):
    with plt.rc_context(RC):
        fig, axes = plt.subplots(3, 1, figsize=(7.5, 9))
        for v, r in u2.items():
            m = r["time_s"] >= 1.0
            _line(axes[0], r["u_mps"][m], (r["r_radps"] - r["r_ref_raw_radps"])[m], v)
            if v in ("inside_cut", "inside_cut_cap"):
                _line(axes[1], r["ay_mps2"][m] / G, r["inside_cut_Nm"][m], v)
                sat = m & r["is_saturated"] & (r["base_torque_Nm"] > 1e-6)
                if sat.any():
                    axes[1].plot(r["ay_mps2"][sat] / G, r["inside_cut_Nm"][sat], "o", ms=3,
                                 color=VARIANT_STYLE[v]["color"], label=f"{VARIANT_STYLE[v]['label']}: saturated")
        axes[0].axhline(0, color=REF, lw=1)
        axes[0].set_xlabel("speed (m/s)")
        axes[0].set_ylabel("r − r_ref (rad/s)")
        axes[0].set_title("Yaw-rate shortfall vs the neutral reference")
        axes[0].legend(loc="lower left")
        axes[1].set_xlabel("lateral acceleration (g)")
        axes[1].set_ylabel("inside-rear cut (Nm)")
        axes[1].set_title("How hard the inside cut is working")
        axes[1].legend(loc="upper left")
        if "oracle" in u2:
            r = u2["oracle"]
            m = r["time_s"] >= 1.0
            _line(axes[2], r["ay_mps2"][m] / G, r["external_yaw_moment_Nm"][m], "oracle",
                  label="oracle request (kp=200 on yaw error)")
        if "off" in u2:
            # the rear Fx difference is not zero with TV off (load transfer, slip), so subtract the
            # TV-off run at the same ay: what is left is the moment the inside cut actually adds
            edges = np.arange(0.0, 16.0, 0.25)
            mid = (edges[:-1] + edges[1:]) / 2
            base = _binnedOn(u2["off"]["ay_mps2"], u2["off"]["tv_yaw_moment_Nm"], u2["off"]["time_s"] >= 1.0, edges)
            for v in ("inside_cut", "inside_cut_cap"):
                if v not in u2:
                    continue
                r = u2[v]
                added = _binnedOn(r["ay_mps2"], r["tv_yaw_moment_Nm"], r["time_s"] >= 1.0, edges) - base
                ok = np.isfinite(added)
                _line(axes[2], mid[ok] / G, added[ok], v, label=f"added by {VARIANT_STYLE[v]['label']} (minus TV off)")
        axes[2].axhline(0, color=REF, lw=1)
        axes[2].set_xlabel("lateral acceleration (g)")
        axes[2].set_ylabel("yaw moment (Nm)")
        axes[2].set_title("Extra yaw moment: delivered by the cut vs requested by the oracle")
        axes[2].legend(loc="upper left")
        _save(fig, path)


def plotU3Yaw(u3, wheelbase_m, ay_lim, path):
    speeds = sorted(u3)
    with plt.rc_context(RC):
        fig, axes = plt.subplots(1, len(speeds), figsize=(max(8.0, 4.2 * len(speeds)), 4.4), squeeze=False)
        for ax, u in zip(axes[0], speeds):
            r_lim = ay_lim / u
            for v, r in u3[u].items():
                m = r["time_s"] >= 0.5
                _line(ax, deg(r["delta_rad"][m]), r["r_radps"][m], v)
                if r["stop_reason"] != "time_up":
                    ax.plot(deg(r["delta_rad"][-1]), r["r_radps"][-1], "x", ms=8, mew=2,
                            color=VARIANT_STYLE[v]["color"])
            d = np.linspace(0, 25, 100)
            ax.plot(d, u * np.tan(np.radians(d)) / wheelbase_m, color=REF, ls="--", lw=1, label="neutral u·tanδ/L")
            ax.axhline(r_lim, color=REF, ls=":", lw=1, label=f"limit ay_lim/u = {r_lim:.2f}")
            ax.set_ylim(0, 1.6 * r_lim)
            ax.set_xlim(0, 25)
            ax.set_title(f"{u:g} m/s")
            ax.set_xlabel("road-wheel steer δ (deg)")
        axes[0][0].set_ylabel("yaw rate r (rad/s)")
        axes[0][0].legend(loc="lower right")
        _save(fig, path, "Constant speed, steer ramp 1°/s — below the dashed line = understeer (x = run stopped)")


def plotU3Kus(u3, u1, wheelbase_m, path):
    with plt.rc_context(RC):
        fig, ax = plt.subplots(figsize=(7.5, 4.4))
        for color, u in zip(SPEED_RAMP, sorted(u3)):
            r = u3[u].get("off")
            if r is None:
                continue
            c, K = kusLocal(r, wheelbase_m, start_s=0.5)
            ax.plot(c / G, kusDegPerG(K), "o-", ms=3, lw=LW, color=color, label=f"steer ramp at {u:g} m/s")
        c, K = kusLocal(u1, wheelbase_m)
        ax.plot(c / G, kusDegPerG(K), "s--", ms=3, lw=1.2, color=TEXT, label="constant radius 9.125 m (U1)")
        ax.axhline(0, color=REF, lw=1)
        ax.set_ylim(-1.0, 2.0)
        ax.text(0.99, 0.98, "near the limit the gradient runs off-scale (front saturates)", transform=ax.transAxes,
                ha="right", va="top", fontsize=8, color=TEXT_2)
        ax.set_xlabel("lateral acceleration (g)")
        ax.set_ylabel("K_us (deg/g)")
        ax.set_title("Understeer gradient by speed (TV off)")
        ax.legend(loc="upper left")
        _save(fig, path)


def plotU3Paths(u3, path):
    speeds = sorted(u3)
    with plt.rc_context(RC):
        fig, axes = plt.subplots(1, len(speeds), figsize=(max(8.0, 4.2 * len(speeds)), 4.4), squeeze=False)
        for ax, u in zip(axes[0], speeds):
            for v, r in u3[u].items():
                _line(ax, r["X_m"], r["Y_m"], v)
            ax.set_aspect("equal", adjustable="datalim")
            ax.set_title(f"{u:g} m/s")
            ax.set_xlabel("X (m)")
        axes[0][0].set_ylabel("Y (m)")
        axes[0][0].legend(loc="best")
        _save(fig, path, "Steer-ramp paths: the radius shrinks as steer grows")


# ---------------------------------------------------------------- plots: corner entry

def plotE1Step(e1, t_step, path):
    keys = sorted(e1)                          # (speed, level)
    speeds = sorted({k[0] for k in keys})
    levels = sorted({k[1] for k in keys})
    with plt.rc_context(RC):
        fig, axes = plt.subplots(len(levels) + 1, len(speeds), figsize=(max(8.0, 4.2 * len(speeds)), 3.0 * (len(levels) + 1)),
                                 squeeze=False, sharex=True)
        for j, u in enumerate(speeds):
            for i, lvl in enumerate(levels):
                ax = axes[i][j]
                runs = e1.get((u, lvl), {})
                for v, r in runs.items():
                    _line(ax, r["time_s"], r["r_radps"], v)
                if "inside_cut" in runs:
                    ax.plot(runs["inside_cut"]["time_s"], runs["inside_cut"]["r_ref_filtered_radps"],
                            color=REF, ls=":", lw=1, label="r_ref (filtered, inside cut)")
                ax.axvspan(t_step, t_step + 0.1, color=GRID, alpha=0.8, lw=0)
                ax.set_title(f"{u:g} m/s, target {lvl:.0%} of limit")
                if j == 0:
                    ax.set_ylabel("yaw rate r (rad/s)")
            ax = axes[-1][j]
            runs = e1.get((u, levels[-1]), {})
            for v, r in runs.items():
                _line(ax, r["time_s"], deg(r["sideslip_rad"]), v)
            ax.axvspan(t_step, t_step + 0.1, color=GRID, alpha=0.8, lw=0)
            ax.set_title(f"{u:g} m/s, {levels[-1]:.0%}: sideslip")
            ax.set_xlabel("time (s)")
            if j == 0:
                ax.set_ylabel("β (deg)")
        axes[0][0].legend(loc="lower right")
        for row in axes:
            for ax in row:
                ax.set_xlim(t_step - 0.3, t_step + 2.5)
        _save(fig, path, "Step steer from straight (shaded: 0.1 s steer input)")


def _groupedBars(ax, groups, variants, values, ylabel, fmt="{:.2f}"):
    width = 0.8 / max(1, len(variants))
    x = np.arange(len(groups))
    for k, v in enumerate(variants):
        vals = np.array([values.get((g, v), math.nan) for g in groups], float)
        pos = x - 0.4 + width * (k + 0.5)
        shown = np.where(np.isfinite(vals), vals, 0.0)
        ax.bar(pos, shown, width * 0.9, color=VARIANT_STYLE[v]["color"], label=VARIANT_STYLE[v]["label"],
               hatch="//" if v == "oracle" else None, edgecolor=SURFACE, linewidth=0.5)
        for p, val in zip(pos, vals):
            if not np.isfinite(val):
                ax.text(p, 0, "n/a", ha="center", va="bottom", fontsize=6, color=TEXT_2, rotation=90)
    ax.set_xticks(x)
    ax.set_xticklabels(groups, fontsize=8)
    ax.set_ylabel(ylabel)


def plotE1Metrics(table, variants, path):
    groups = sorted({(row["speed"], row["level"]) for row in table})
    labels = [f"{u:g} m/s\n{lvl:.0%}" for u, lvl in groups]
    with plt.rc_context(RC):
        fig, axes = plt.subplots(1, 3, figsize=(14, 4.2))
        for ax, (key, ylabel) in zip(axes, [("overshoot_pct", "yaw overshoot (%)"),
                                            ("settle_s", "settling time ±5% (s)"),
                                            ("beta_peak_deg", "peak |β| (deg)")]):
            values = {(f"{row['speed']:g} m/s\n{row['level']:.0%}", row["variant"]): row.get(key, math.nan)
                      for row in table}
            _groupedBars(ax, labels, variants, values, ylabel)
        axes[0].legend(loc="upper left")
        _save(fig, path, "Step-steer entry metrics (n/a = run stopped)")


# ---------------------------------------------------------------- plots: oversteer

def plotO1(o1, ay_lim, path, window_s=5.0):
    t_end = max(r["time_s"][-1] for r in o1.values())
    t0 = max(1.0, min(r["time_s"][-1] for r in o1.values()) - window_s)
    with plt.rc_context(RC):
        fig, axes = plt.subplots(5, 1, figsize=(8, 12), sharex=True)
        for v, r in o1.items():
            m = r["time_s"] >= t0
            t = r["time_s"][m]
            af, ar = axleSlip(r)
            _line(axes[0], t, r["r_radps"][m], v)
            _line(axes[1], t, deg(r["delta_rad"][m]), v)
            _line(axes[2], t, r["inside_cut_Nm"][m], v)
            _line(axes[3], t, deg(r["sideslip_rad"][m]), v)
            _line(axes[4], t, deg(af - ar)[m], v)
            if r["stop_reason"] != "time_up":
                for ax, ch in zip(axes, [r["r_radps"], deg(r["delta_rad"]), r["inside_cut_Nm"],
                                         deg(r["sideslip_rad"]), deg(af - ar)]):
                    ax.plot(r["time_s"][-1], ch[-1], "x", ms=8, mew=2, color=VARIANT_STYLE[v]["color"])
        if "inside_cut" in o1:
            r = o1["inside_cut"]
            m = r["time_s"] >= t0
            axes[0].plot(r["time_s"][m], r["r_ref_raw_radps"][m], color=VARIANT_STYLE["inside_cut"]["color"],
                         ls=":", lw=1.2, label="r_ref uncapped (inside cut)")
        ref = o1.get("off") or next(iter(o1.values()))
        m = ref["time_s"] >= t0
        axes[0].plot(ref["time_s"][m], ay_lim / ref["u_mps"][m], color=REF, ls="--", lw=1,
                     label="achievable ay_lim/u")
        axes[0].set_ylim(0, 3.0)
        axes[0].set_ylabel("yaw rate (rad/s)")
        axes[0].legend(loc="upper left", ncol=2)
        axes[1].set_ylabel("driver steer δ (deg)")
        axes[2].set_ylabel("inside-rear cut (Nm)")
        axes[3].set_ylabel("sideslip β (deg)")
        axes[4].axhline(0, color=REF, lw=1)
        axes[4].set_ylabel("α_f − α_r (deg)\n+ understeer / − oversteer")
        axes[4].set_xlabel("time (s)")
        axes[4].set_xlim(t0, t_end + 0.1)
        _save(fig, path, "Constant-radius ramp to the limit, last seconds (x = run stopped)")


def plotO1Phase(o1, path):
    with plt.rc_context(RC):
        fig, ax = plt.subplots(figsize=(7, 5))
        for v, r in o1.items():
            m = r["time_s"] >= 1.0
            _line(ax, r["r_radps"][m], deg(r["sideslip_rad"][m]), v)
            marker = "x" if r["stop_reason"] == "spun" else ("s" if r["stop_reason"] == "off the circle" else "o")
            ax.plot(r["r_radps"][-1], deg(r["sideslip_rad"][-1]), marker, ms=8, mew=2,
                    color=VARIANT_STYLE[v]["color"])
        ax.axhline(0, color=REF, lw=1)
        ax.set_xlabel("yaw rate r (rad/s)")
        ax.set_ylabel("sideslip β (deg)")
        ax.set_title("β–r phase portrait, constant-radius ramp (x spun · ■ off circle · ● time up)")
        ax.legend(loc="lower left")
        _save(fig, path)


def eventLabel(event, severity):
    return {"yaw_kick": f"+{severity:.2g} rad/s", "rear_grip": f"−{100 * severity:.0f}% rear grip",
            "rear_torque": f"+{severity:.0f} Nm rear", "lift_off": f"{severity:.2g} s lift"}.get(event, f"{severity:g}")


def plotO2Event(event, runs, t_event, durations, sizes, path):
    """runs: {(driver, variant): result}; durations/sizes: {driver: value}"""
    drivers = ["fixed", "circle"]
    with plt.rc_context(RC):
        fig, axes = plt.subplots(4, 2, figsize=(11, 10.5), sharex=True)
        for j, drv in enumerate(drivers):
            for (d, v), r in runs.items():
                if d != drv:
                    continue
                r_ss, beta_ss = baseline(r, t_event)
                t = r["time_s"] - t_event
                _line(axes[0][j], t, r["r_radps"] - r_ss, v)
                _line(axes[1][j], t, deg(r["sideslip_rad"] - beta_ss), v)
                _line(axes[2][j], t, axleUtil(r)[1], v)
                _line(axes[3][j], t, r["inside_cut_Nm"], v)
                if r["stop_reason"] != "time_up":
                    axes[0][j].plot(t[-1], r["r_radps"][-1] - r_ss, "x", ms=8, mew=2, color=VARIANT_STYLE[v]["color"])
            axes[0][j].axhspan(-0.05, 0.05, color=GRID, alpha=0.9, lw=0)
            axes[2][j].axhline(1.0, color=REF, ls="--", lw=1)
            for ax in axes[:, j]:
                ax.axvspan(0, durations[drv], color=GRID, alpha=0.5, lw=0)
                ax.set_xlim(-0.5, 5.0)
            axes[0][j].set_title(("fixed steer (plant + TV)" if drv == "fixed" else "pure pursuit (driver + TV)")
                                 + f", {eventLabel(event, sizes[drv])}")
            axes[3][j].set_xlabel("time since event (s)")
        axes[0][0].set_ylabel("Δr (rad/s)\nband ±0.05")
        axes[1][0].set_ylabel("Δβ (deg)")
        axes[2][0].set_ylabel("rear grip use")
        axes[3][0].set_ylabel("inside-rear cut (Nm)")
        axes[0][0].legend(loc="upper right")
        _save(fig, path, f"Disturbance: {event} (shaded: event window, x = run stopped)")


def plotO2Summary(table, variants, path):
    rows = sorted({(row["event"], row["driver"]) for row in table})
    cell = {(row["event"], row["driver"], row["variant"]): row for row in table}
    size = {(row["event"], row["driver"]): row["severity"] for row in table}
    with plt.rc_context(RC):
        fig, axes = plt.subplots(1, 2, figsize=(13, 0.55 * len(rows) + 2.2))
        for ax, (key, title, unit) in zip(axes, [("settle_s", "Yaw settling time", "s"),
                                                  ("peak_dbeta_deg", "Peak sideslip change", "deg")]):
            grid = np.full((len(rows), len(variants)), np.nan)
            for i, (ev, drv) in enumerate(rows):
                for j, v in enumerate(variants):
                    row = cell.get((ev, drv, v))
                    if row is not None and row["recovered"]:
                        grid[i, j] = row.get(key, np.nan)
            vmax = np.nanmax(grid) if np.isfinite(grid).any() else 1.0
            ax.imshow(np.where(np.isfinite(grid), grid, np.nan), cmap=SEQ_BLUE, vmin=0, vmax=vmax, aspect="auto")
            ax.grid(False)
            for i, (ev, drv) in enumerate(rows):
                for j, v in enumerate(variants):
                    row = cell.get((ev, drv, v))
                    if row is None:
                        continue
                    if not row["recovered"]:
                        ax.add_patch(plt.Rectangle((j - 0.5, i - 0.5), 1, 1, facecolor="#f0efec",
                                                   edgecolor=SURFACE, hatch="xx", lw=0))
                        text, color = row["stop"].upper() if row["stop"] != "time_up" else "NO RECOVERY", TEXT
                    else:
                        val = grid[i, j]
                        text = f"{val:.2f}" if np.isfinite(val) else "n/a"
                        color = SURFACE if np.isfinite(val) and val > 0.6 * vmax else TEXT
                    if row.get("cut_while_rear_saturated"):
                        text += " ▲"
                    ax.text(j, i, text, ha="center", va="center", fontsize=7.5, color=color)
            ax.set_xticks(range(len(variants)))
            ax.set_xticklabels([VARIANT_STYLE[v]["label"] for v in variants], fontsize=8, rotation=15)
            ax.set_yticks(range(len(rows)))
            ax.set_yticklabels([f"{eventLabel(ev, size[(ev, drv)])} · {drv}" for ev, drv in rows], fontsize=8)
            ax.set_title(f"{title} ({unit}); darker = worse")
        fig.text(0.01, 0.01, "▲ = inside cut active while the rear axle was at its grip limit "
                 "(cutting rear drive frees rear lateral grip, so this is not automatically worse)", fontsize=8, color=TEXT_2)
        _save(fig, path, "Disturbance recovery at 90% of the limit")


def plotO3(margins, variants, path):
    """margins: {(kind, variant): {"lo":..., "hi":..., "never_failed": bool}}"""
    kinds = [("rear_grip", "critical rear grip loss (%)", 100.0), ("rear_torque", "critical rear torque pulse (Nm)", 1.0)]
    with plt.rc_context(RC):
        fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
        for ax, (kind, ylabel, scale) in zip(axes, kinds):
            for k, v in enumerate(variants):
                m = margins.get((kind, v))
                if m is None:
                    continue
                val = (m["lo"] + m["hi"]) / 2 * scale if not m["never_failed"] else m["hi"] * scale
                ax.bar(k, val, 0.7, color=VARIANT_STYLE[v]["color"], edgecolor=SURFACE,
                       hatch="//" if (m["never_failed"] or v == "oracle") else None)
                label = (f"≥ {val:.0f}" if m["never_failed"] else f"{val:.0f}") if scale == 1.0 else \
                        (f"≥ {val:.0f}" if m["never_failed"] else f"{val:.1f}")
                ax.text(k, val, label, ha="center", va="bottom", fontsize=8, color=TEXT)
            ax.set_xticks(range(len(variants)))
            ax.set_xticklabels([VARIANT_STYLE[v]["label"] for v in variants], fontsize=8, rotation=15)
            ax.set_ylabel(ylabel)
            ax.set_title(ylabel.split(" (")[0].capitalize() + " before the car stops recovering")
        _save(fig, path, "Spin margin at 90% of the limit, pure pursuit (higher = more stable; ≥ = never failed)")
