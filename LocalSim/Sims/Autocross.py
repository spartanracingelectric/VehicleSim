# Sweep settings: turn each sweep on/off and set its range as (start, end, step),
# end included. Each sweep re-runs the full lap sim once per value, for each car.
RUN_POWER_LIMIT_SWEEP = False
POWER_LIMIT_RANGE = (10000.0, 90000.0, 1000.0)   # W

RUN_CD_SWEEP = True
CD_RANGE = (0.5, 3.0, 0.05)

RUN_CL_SWEEP = True
CL_RANGE = (1.0, 15.0, 0.05)                     # cd scales with cl to hold the car's default L/D

import sys
from pathlib import Path
SCRIPT_DIR = Path(__file__).resolve().parent
RESOURCES_DIR = SCRIPT_DIR.parent / "Resources"
OUTPUT_DIR = SCRIPT_DIR.parent / "Outputs" / "Autocross"
sys.path.insert(0, str(RESOURCES_DIR / "Car_Setups"))
sys.path.insert(0, str(SCRIPT_DIR))  # Skidpad.py, in this same folder

import numpy as np
import matplotlib.pyplot as plt
from SR_17 import SR_17
from SR_18 import SR_18
from Skidpad import available_forces, corner_speed_feasible

# Course centerline (ft), traced from Autocross_2024.png by digitize_track.py (same folder)
TRACK_CSV = RESOURCES_DIR / "Tracks" / "Autocross_2024" / "track_centerline.csv"

def sweep_values(start, end, step):
    """Values from start to end (inclusive) in steps of step, rounded to clear float noise."""
    return np.round(np.arange(start, end + step / 1000.0, step), 6)

# attr -> (on/off, values, x-axis unit divisor, x-axis label, plot title fragment)
SWEEP_PARAMS = {
    "power_limit": (RUN_POWER_LIMIT_SWEEP, sweep_values(*POWER_LIMIT_RANGE), 1000.0, "Power Limit [kW]", "Power Limit"),
    "cd":          (RUN_CD_SWEEP, sweep_values(*CD_RANGE), 1.0, "Drag Coefficient (Cd)", "Drag Coefficient"),
    "cl":          (RUN_CL_SWEEP, sweep_values(*CL_RANGE), 1.0, "Lift Coefficient (Cl)", "Lift Coefficient"),
}

def load_track(path=TRACK_CSV):
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    return data[:, 0], data[:, 1]

def path_curvature(x, y):
    """Signed curvature at each waypoint of the closed loop, by finite differences
    (wrapping a few points across the start/finish so it stays smooth there)."""
    pad = 5
    xp = np.concatenate([x[-pad:], x, x[:pad]])
    yp = np.concatenate([y[-pad:], y, y[:pad]])
    dx, dy = np.gradient(xp), np.gradient(yp)
    ddx, ddy = np.gradient(dx), np.gradient(dy)
    kappa = (dx * ddy - dy * ddx) / (dx**2 + dy**2)**1.5
    return kappa[pad:-pad]

def max_corner_speed_bisect(car, radius, v_max_search=60.0, iters=40):
    """Fastest steady speed (m/s) on this radius, by bisection on Skidpad's
    corner_speed_feasible. Returns v_max_search if even that speed is feasible."""
    if not corner_speed_feasible(car, v_max_search, radius):
        lo, hi = 0.0, v_max_search
        for _ in range(iters):
            mid = 0.5 * (lo + hi)
            if corner_speed_feasible(car, mid, radius):
                lo = mid
            else:
                hi = mid
        return lo
    return v_max_search

def max_accel(car, v, radius):
    """Max acceleration (m/s^2) at speed v while holding this radius: friction-circle
    grip left after the lateral demand, capped by the drive force, minus drag and
    rolling resistance. The friction circle is sized by peak lateral grip."""
    g = 9.81
    f_grip, f_long_drivetrain, f_drag, f_rr = available_forces(car, v, radius)
    f_lat_required = car.mass * v**2 / radius
    f_long_tire = np.sqrt(max(0.0, f_grip**2 - f_lat_required**2))
    f_drive = min(f_long_tire, f_long_drivetrain)
    # SR_17 calls this rotational_inertia_factor_front, SR_18 rotational_inertia_factor
    rif = getattr(car, 'rotational_inertia_factor_front', getattr(car, 'rotational_inertia_factor', 1.0))
    m_eff = car.mass * rif
    return (f_drive - f_drag - f_rr) / m_eff

def max_decel(car, v, radius):
    """Max braking deceleration (m/s^2) at speed v while holding this radius:
    friction-circle grip left after the lateral demand, plus drag and rolling
    resistance. Assumes the brakes can always reach the tire limit (no brake model)."""
    g = 9.81
    f_grip, _, f_drag, f_rr = available_forces(car, v, radius)
    f_lat_required = car.mass * v**2 / radius
    f_long_tire = np.sqrt(max(0.0, f_grip**2 - f_lat_required**2))
    rif = getattr(car, 'rotational_inertia_factor_front', getattr(car, 'rotational_inertia_factor', 1.0))
    m_eff = car.mass * rif
    return (f_long_tire + f_drag + f_rr) / m_eff

def simulate_lap(car, x, y, ds, rounds=3, v_floor=1.0):
    """Quasi-steady-state point-mass lap sim on a closed loop. Each waypoint's speed is
    capped by the steady-state cornering limit at its curvature, by how fast the car
    could accelerate up to it from the previous point, and by how fast it can brake for
    the next point. Repeated `rounds` times, since on a closed loop each pass depends
    on the other.

    Returns lap_time (s), v (m/s per waypoint), radius (m per waypoint), dt (s per step).
    """
    n = len(x)
    kappa = path_curvature(x, y)
    radius = np.clip(1.0 / np.clip(np.abs(kappa), 1e-6, None), 3.0, 5000.0)  # clamped to 3-5000 m

    v_corner = np.array([max_corner_speed_bisect(car, r) for r in radius])
    v = v_corner.copy()

    for _ in range(rounds):
        # Forward pass: limit to what the car can accelerate to from the previous point
        for i in range(n):
            prev = i - 1
            a = max_accel(car, v[prev], radius[prev])
            v_reachable = np.sqrt(max(v[prev]**2 + 2.0 * a * ds, v_floor**2))
            v[i] = min(v[i], v_reachable)

        # Backward pass: limit to what the car can brake from in time for the next point
        for i in range(n - 1, -1, -1):
            nxt = (i + 1) % n
            a = max_decel(car, v[nxt], radius[nxt])
            v_needed = np.sqrt(max(v[nxt]**2 + 2.0 * a * ds, v_floor**2))
            v[i] = min(v[i], v_needed)

    v_avg = 0.5 * (v + np.roll(v, -1))
    dt = ds / v_avg
    lap_time = dt.sum()

    return lap_time, v, radius, dt

def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    x_ft, y_ft = load_track()
    x, y = x_ft * 0.3048, y_ft * 0.3048  # ft -> m
    n = len(x)
    perimeter = np.sum(np.hypot(np.diff(np.append(x, x[0])), np.diff(np.append(y, y[0]))))
    ds = perimeter / n

    cars = {
        "SR_17": ("SR_17 (Single-Motor RWD)", SR_17),
        "SR_18": ("SR_18 (4-Motor AWD)", SR_18),
    }

    for key, (label, CarClass) in cars.items():
        car = CarClass()
        lap_time, v, radius, dt = simulate_lap(car, x, y, ds)

        print(f"--- FSAE Autocross Lap Simulation (2024 course): {label} ---")
        print(f"Track Length:  {perimeter:.1f} m ({perimeter / 0.3048:.0f} ft)")
        print(f"Lap Time:      {lap_time:.3f} s")
        print(f"Top Speed:     {v.max() * 3.6:.1f} km/h")
        print(f"Min Corner Speed: {v.min() * 3.6:.1f} km/h (radius {radius[np.argmin(v)] / 0.3048:.1f} ft)")

        s = np.arange(n) * ds

        fig, axs = plt.subplots(2, 1, figsize=(16, 8))
        axs[0].plot(s, v * 3.6)
        axs[0].set_xlabel("Distance Along Track [m]")
        axs[0].set_ylabel("Speed [km/h]")
        axs[0].set_title(f"{key} Autocross Lap Speed Trace")
        axs[0].grid(True)

        sc = axs[1].scatter(x_ft, y_ft, c=v * 3.6, cmap="turbo", s=6)
        axs[1].set_aspect("equal")
        axs[1].set_xlabel("x [ft]")
        axs[1].set_ylabel("y [ft]")
        axs[1].set_title(f"{key} Speed Around the Track")
        fig.colorbar(sc, ax=axs[1], label="Speed [km/h]")

        plt.tight_layout()
        graph_path = OUTPUT_DIR / f"{key}_Autocross_lap_speed.png"
        plt.savefig(graph_path, dpi=150)
        plt.close(fig)
        print(f"Saved speed trace graph to {graph_path}")

        # Sweeps that are on: each varies one setting (everything else at the car's
        # default) and saves its own lap-time graph. For the cl sweep, cd scales with cl
        # to hold this car's default L/D, so added downforce isn't free.
        cd_per_cl = car.cd / car.cl

        for attr, (run, values, unit_div, xlabel, title_frag) in SWEEP_PARAMS.items():
            if not run:
                continue
            lap_times = []
            for val in values:
                sweep_car = CarClass()
                setattr(sweep_car, attr, val)
                if attr == "cl":
                    sweep_car.cd = val * cd_per_cl
                sweep_time, _, _, _ = simulate_lap(sweep_car, x, y, ds)
                lap_times.append(sweep_time)

            plt.figure(figsize=(8, 5))
            plt.plot(values / unit_div, lap_times, marker='o', markersize=3)
            plt.xlabel(xlabel)
            plt.ylabel("Lap Time (s)")
            plt.title(f"{key} Autocross Lap Time vs. {title_frag}")
            plt.grid(True)
            sweep_graph_path = OUTPUT_DIR / f"{key}_Autocross_{attr}_sweep.png"
            plt.savefig(sweep_graph_path, dpi=150)
            plt.close()
            print(f"Saved {attr} sweep graph to {sweep_graph_path}")

        print()

if __name__ == "__main__":
    main()
