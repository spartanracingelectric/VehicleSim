# Sweep settings: turn each sweep on/off and set its range as (start, end, step),
# end included. Each sweep re-runs the full skidpad sim once per value, for each car.
RUN_TIRE_PRESSURE_SWEEP = True
TIRE_PRESSURE_RANGE = (7.0, 12.0, 0.25)    # psi; tire data only covers 8-14 psi, so
                                           # values below 8 give the same result as 8

import sys
from pathlib import Path
RESOURCES_DIR = Path(__file__).resolve().parent.parent / "Resources"
sys.path.insert(0, str(RESOURCES_DIR / "Car_Setups"))

import numpy as np
from SR_17 import SR_17
from SR_18 import SR_18

def aero_downforce(car, v, aero_yaw_derate=1.0):
    """Total aero downforce (N) at speed v, scaled by aero_yaw_derate (see
    aero_yaw_derate_factor)."""
    return 0.5 * car.air_density * car.cl * car.frontal_area * v**2 * aero_yaw_derate

def axle_normal_loads(car, v, radius, aero_yaw_derate=1.0):
    """Per-corner normal loads in N (front outer, front inner, rear outer, rear inner):
    static weight + aero downforce + lateral load transfer (split front/rear by
    roll_stiffness_front), each clamped at zero.

    aero_yaw_derate (0-1) scales downforce only. Defaults to 1.0 (no loss); only the
    skidpad circle passes a real value.
    """
    g = 9.81
    static_front_bias = 1.0 - car.static_rear_bias
    aero_front_bias = 1.0 - car.aero_rear_bias

    a_lat = v**2 / radius
    f_down = aero_downforce(car, v, aero_yaw_derate)

    f_z_front_total = car.mass * g * static_front_bias + f_down * aero_front_bias
    f_z_rear_total = car.mass * g * car.static_rear_bias + f_down * car.aero_rear_bias

    transfer_front = car.roll_stiffness_front * car.mass * a_lat * car.cg_height / car.track_width
    transfer_rear = (1.0 - car.roll_stiffness_front) * car.mass * a_lat * car.cg_height / car.track_width

    f_z_front_outer = max(0.0, f_z_front_total / 2.0 + transfer_front)
    f_z_front_inner = max(0.0, f_z_front_total / 2.0 - transfer_front)
    f_z_rear_outer = max(0.0, f_z_rear_total / 2.0 + transfer_rear)
    f_z_rear_inner = max(0.0, f_z_rear_total / 2.0 - transfer_rear)

    return f_z_front_outer, f_z_front_inner, f_z_rear_outer, f_z_rear_inner

def available_forces(car, v, radius, aero_yaw_derate=1.0):
    """Whole-car forces in N at speed v on this radius: (lateral grip, drivetrain
    force, drag, rolling resistance).

    Grip is the sum of each corner's peak lateral force at its own load, so uneven
    left/right loading costs grip (tire load sensitivity). max_accel/max_decel here and
    in Autocross.py/Endurance.py treat it as one whole-car friction circle.
    """
    g = 9.81
    num_driven_motors = getattr(car, 'num_motors', 1)

    f_z_front_outer, f_z_front_inner, f_z_rear_outer, f_z_rear_inner = axle_normal_loads(car, v, radius, aero_yaw_derate)
    f_grip_available = (car.tire_peak_lateral(f_z_front_outer) + car.tire_peak_lateral(f_z_front_inner)
                         + car.tire_peak_lateral(f_z_rear_outer) + car.tire_peak_lateral(f_z_rear_inner))

    f_drag = 0.5 * car.air_density * car.cd * car.frontal_area * v**2
    # Rolling resistance on total tire load (weight + downforce)
    f_rr = car.crr * (car.mass * g + aero_downforce(car, v, aero_yaw_derate))

    # Drive force limit: motor torque at the RPM set by final_drive_ratio, with total
    # torque x speed across all motors capped at power_limit. drivetrain_efficiency is
    # applied after, so this acts as a battery-side cap (inverter losses not modeled).
    # A car without power_limit is left uncapped.
    wheel_rad_s = v / car.wheel_radius
    motor_rpm = (wheel_rad_s * 60.0) / (2.0 * np.pi) * car.final_drive_ratio
    motor_omega = motor_rpm * (2.0 * np.pi) / 60.0
    torque_avail = car.get_motor_torque(motor_rpm)
    power_limit = getattr(car, 'power_limit', None)
    if power_limit is not None and motor_omega > 1e-6:
        power_draw_total = num_driven_motors * torque_avail * motor_omega
        if power_draw_total > power_limit:
            torque_avail = power_limit / (num_driven_motors * motor_omega)
    wheel_torque = torque_avail * car.final_drive_ratio * car.drivetrain_efficiency
    f_long_achievable = num_driven_motors * wheel_torque / car.wheel_radius

    return f_grip_available, f_long_achievable, f_drag, f_rr

def corner_speed_feasible(car, v, radius, aero_yaw_derate=1.0):
    """Whether the car can hold a steady circle of this radius at speed v.

    Checked per axle: each axle's combined lateral + longitudinal force must fit within
    its own grip, and its share of the drive force needed to hold speed (drag + rolling
    resistance) must be within what its motors can deliver.
      - Lateral force splits front/rear by static weight distribution (what the
        steady-state yaw moment balance about the CG gives).
      - Longitudinal force splits by torque_split_front (all rear for a car without
        one, i.e. RWD), so a torque split can overload one axle.
    """
    if v <= 0.0:
        return True

    _, f_long_achievable, f_drag, f_rr = available_forces(car, v, radius, aero_yaw_derate)
    f_z_front_outer, f_z_front_inner, f_z_rear_outer, f_z_rear_inner = axle_normal_loads(car, v, radius, aero_yaw_derate)

    # Per-axle grip: each corner's peak lateral force at its own load
    f_grip_front = car.tire_peak_lateral(f_z_front_outer) + car.tire_peak_lateral(f_z_front_inner)
    f_grip_rear = car.tire_peak_lateral(f_z_rear_outer) + car.tire_peak_lateral(f_z_rear_inner)

    static_front_bias = 1.0 - car.static_rear_bias
    f_lat_required = car.mass * v**2 / radius
    f_lat_front_required = f_lat_required * static_front_bias
    f_lat_rear_required = f_lat_required * car.static_rear_bias

    torque_split_front = getattr(car, 'torque_split_front', 0.0)
    f_long_required = f_drag + f_rr
    f_long_front_required = f_long_required * torque_split_front
    f_long_rear_required = f_long_required * (1.0 - torque_split_front)
    f_long_front_achievable = f_long_achievable * torque_split_front
    f_long_rear_achievable = f_long_achievable * (1.0 - torque_split_front)

    f_combined_front_required = np.hypot(f_lat_front_required, f_long_front_required)
    f_combined_rear_required = np.hypot(f_lat_rear_required, f_long_rear_required)

    return (f_combined_front_required <= f_grip_front and f_long_front_required <= f_long_front_achievable
            and f_combined_rear_required <= f_grip_rear and f_long_rear_required <= f_long_rear_achievable)

def max_corner_speed_bisect(car, radius, aero_yaw_derate=1.0, v_max_search=60.0, iters=40):
    """Fastest steady speed (m/s) on this radius, by bisection on corner_speed_feasible.
    Returns v_max_search if even that speed is feasible."""
    if not corner_speed_feasible(car, v_max_search, radius, aero_yaw_derate):
        lo, hi = 0.0, v_max_search
        for _ in range(iters):
            mid = 0.5 * (lo + hi)
            if corner_speed_feasible(car, mid, radius, aero_yaw_derate):
                lo = mid
            else:
                hi = mid
        return lo
    return v_max_search

def max_accel(car, v, radius, aero_yaw_derate=1.0):
    """Max acceleration (m/s^2) at speed v while holding this radius: friction-circle
    grip left after the lateral demand, capped by the drive force, minus drag and
    rolling resistance. The friction circle is sized by peak lateral grip."""
    f_grip, f_long_drivetrain, f_drag, f_rr = available_forces(car, v, radius, aero_yaw_derate)
    f_lat_required = car.mass * v**2 / radius
    f_long_tire = np.sqrt(max(0.0, f_grip**2 - f_lat_required**2))
    f_drive = min(f_long_tire, f_long_drivetrain)
    # SR_17 calls this rotational_inertia_factor_front, SR_18 rotational_inertia_factor
    rif = getattr(car, 'rotational_inertia_factor_front', getattr(car, 'rotational_inertia_factor', 1.0))
    m_eff = car.mass * rif
    return (f_drive - f_drag - f_rr) / m_eff

def max_decel(car, v, radius, aero_yaw_derate=1.0):
    """Max braking deceleration (m/s^2) at speed v while holding this radius:
    friction-circle grip left after the lateral demand, plus drag and rolling
    resistance. Assumes the brakes can always reach the tire limit (no brake model)."""
    f_grip, _, f_drag, f_rr = available_forces(car, v, radius, aero_yaw_derate)
    f_lat_required = car.mass * v**2 / radius
    f_long_tire = np.sqrt(max(0.0, f_grip**2 - f_lat_required**2))
    rif = getattr(car, 'rotational_inertia_factor_front', getattr(car, 'rotational_inertia_factor', 1.0))
    m_eff = car.mass * rif
    return (f_long_tire + f_drag + f_rr) / m_eff

# Downforce loss with body slip angle beta: downforce x cos(beta)^AERO_YAW_EXPONENT.
# An estimate, not measured for this car; real losses are likely larger past ~10-15 deg.
AERO_YAW_EXPONENT = 26.0

def aero_yaw_derate_factor(beta_rad, exponent=AERO_YAW_EXPONENT):
    """Downforce multiplier (0-1) at body slip angle beta_rad (rad)."""
    return max(0.0, np.cos(beta_rad)) ** exponent

# Course geometry from FSAE Rules 2027 DRAFT, D.10.1 Skidpad Layout (pg. 139)
CENTER_SPACING = 18.25    # m, distance between the two circle centers
INNER_DIAMETER = 15.25    # m, inner circle diameter
OUTER_DIAMETER = 21.25    # m, outer circle diameter
PATH_WIDTH = (OUTER_DIAMETER - INNER_DIAMETER) / 2.0     # 3.0 m driving corridor
PATH_RADIUS = (INNER_DIAMETER + OUTER_DIAMETER) / 4.0     # 9.125 m driven line (corridor center). Equals
                                                           # CENTER_SPACING / 2, so the two circles touch at
                                                           # one point: the figure-eight crossover
GATE_STRAIGHT_LENGTH = 10.0  # m, entry/exit straight length (not set by the rules)

def build_skidpad_path(ds=0.05):
    """Builds the figure-eight waypoints from the D.10.1 geometry, in D.10.2 order:
    entry straight, 2 right-circle laps (1st untimed, 2nd timed), 2 left-circle laps
    (same), then an exit straight in the same direction as entry. The circle centers
    sit on the x-axis, touching at the origin, and the car crosses the origin heading +y.

    Returns x, y (m), radius (m per waypoint; straights use 1e9), ds_arr (m between
    waypoints, length len(x)-1), and bounds: segment name -> (start, end) waypoint
    indices, inclusive, for "entry", "right_lap1", "right_lap2", "left_lap3",
    "left_lap4", "exit".
    """
    STRAIGHT_RADIUS = 1.0e9  # effectively straight

    def straight_points(y0, y1):
        n = max(2, int(round(abs(y1 - y0) / ds)) + 1)
        y = np.linspace(y0, y1, n)
        return np.zeros(n), y

    def arc_points(center_x, theta0, dtheta):
        n = max(2, int(round(abs(dtheta) * PATH_RADIUS / ds)) + 1)
        theta = np.linspace(theta0, theta0 + dtheta, n)
        return center_x + PATH_RADIUS * np.cos(theta), PATH_RADIUS * np.sin(theta)

    x_e, y_e = straight_points(-GATE_STRAIGHT_LENGTH, 0.0)
    # Right circle: center (+R, 0), starting at theta=pi and going clockwise
    x_r1, y_r1 = arc_points(PATH_RADIUS, np.pi, -2.0 * np.pi)
    x_r2, y_r2 = arc_points(PATH_RADIUS, np.pi, -2.0 * np.pi)
    # Left circle: center (-R, 0), starting at theta=0 and going counterclockwise
    x_l3, y_l3 = arc_points(-PATH_RADIUS, 0.0, 2.0 * np.pi)
    x_l4, y_l4 = arc_points(-PATH_RADIUS, 0.0, 2.0 * np.pi)
    x_x, y_x = straight_points(0.0, GATE_STRAIGHT_LENGTH)

    segments = [
        ("entry", x_e, y_e, STRAIGHT_RADIUS),
        ("right_lap1", x_r1, y_r1, PATH_RADIUS),
        ("right_lap2", x_r2, y_r2, PATH_RADIUS),
        ("left_lap3", x_l3, y_l3, PATH_RADIUS),
        ("left_lap4", x_l4, y_l4, PATH_RADIUS),
        ("exit", x_x, y_x, STRAIGHT_RADIUS),
    ]

    x_parts, y_parts, r_parts = [], [], []
    bounds = {}
    cursor = 0
    for i, (name, xs, ys, r) in enumerate(segments):
        if i > 0:
            # Skip each segment's first point: it duplicates the previous segment's last
            xs, ys = xs[1:], ys[1:]
        seg_start = 0 if i == 0 else cursor - 1
        x_parts.append(xs); y_parts.append(ys); r_parts.append(np.full(len(xs), r))
        cursor += len(xs)
        bounds[name] = (seg_start, cursor - 1)

    x = np.concatenate(x_parts)
    y = np.concatenate(y_parts)
    radius = np.concatenate(r_parts)
    ds_arr = np.hypot(np.diff(x), np.diff(y))

    return x, y, radius, ds_arr, bounds

def solve_circle_speed_with_yaw_aero(car, radius, iters=5):
    """Circle speed including aero yaw loss. Corner speed depends on downforce,
    downforce depends on body slip angle, and slip angle depends on corner speed, so
    this alternates between the bisection speed solve and YawPlane's slip angle solve
    until they settle.

    Returns (v in m/s, downforce multiplier, body slip angle in rad).
    """
    # Imported here, not at the top, since YawPlane imports from this module
    sys.path.insert(0, str(RESOURCES_DIR / "VehicleDynamics"))
    from YawPlane import max_corner_speed_yaw_plane

    # YawPlane's solver needs a warm start near the grip limit, so reuse its speed
    # sweep (which warm-starts each step) up to just above v
    dv = 0.05
    derate = 1.0
    beta = 0.0
    v = max_corner_speed_bisect(car, radius, aero_yaw_derate=derate)
    for _ in range(iters):
        _, yaw_result = max_corner_speed_yaw_plane(car, radius, v_search_max=v + dv, dv=dv)
        if yaw_result is None:
            break  # no solution: keep the last converged values
        beta = np.arctan2(yaw_result['vy'], v)
        derate = aero_yaw_derate_factor(beta)
        v = max_corner_speed_bisect(car, radius, aero_yaw_derate=derate)

    return v, derate, beta

def simulate_skidpad_run(car, ds=0.05, rounds=3):
    """Simulates one skidpad attempt (one driver, entry to exit, standing start) with
    the same forward/backward quasi-steady-state point-mass method as Autocross.py.

    Each waypoint's speed is capped by the steady-state cornering limit (on the circle,
    including aero yaw loss), by how fast the car could accelerate up to it, and by how
    fast it can brake for the next point. Corner transitions, including the crossover,
    are instant: no transient or suspension lag.

    Returns a dict: x, y, v (m/s), radius, ds_arr, bounds (see build_skidpad_path),
    right_lap_time/left_lap_time (s, the two timed laps), corrected_time (s, their
    average per D.10.4, no cone penalties), total_time (s, entry to exit), max_speed
    (m/s), aero_yaw_derate (circle downforce multiplier), body_slip_deg (circle body
    slip angle, deg).
    """
    x, y, radius, ds_arr, bounds = build_skidpad_path(ds=ds)
    n = len(x)

    # Only two radii: the straights (car points where it's going, no aero loss) and
    # the circle
    v_corner_straight = max_corner_speed_bisect(car, 1.0e9)
    v_corner_circle, derate_circle, beta_circle = solve_circle_speed_with_yaw_aero(car, PATH_RADIUS)

    is_circle = radius <= 1.0e6
    v_corner = np.where(is_circle, v_corner_circle, v_corner_straight)
    derate_arr = np.where(is_circle, derate_circle, 1.0)

    v = v_corner.copy()
    v[0] = 0.0  # standing start

    for _ in range(rounds):
        for i in range(1, n):
            a = max_accel(car, v[i - 1], radius[i - 1], derate_arr[i - 1])
            v[i] = min(v[i], np.sqrt(max(v[i - 1]**2 + 2.0 * a * ds_arr[i - 1], 0.0)))
        for i in range(n - 2, -1, -1):
            a = max_decel(car, v[i + 1], radius[i + 1], derate_arr[i + 1])
            v[i] = min(v[i], np.sqrt(max(v[i + 1]**2 + 2.0 * a * ds_arr[i], 0.0)))
        v[0] = 0.0

    v_avg = 0.5 * (v[:-1] + v[1:])
    dt_arr = ds_arr / v_avg

    def segment_time(name):
        s0, s1 = bounds[name]
        return float(dt_arr[s0:s1].sum())

    right_lap_time = segment_time("right_lap2")
    left_lap_time = segment_time("left_lap4")
    corrected_time = (right_lap_time + left_lap_time) / 2.0  # D.10.4, no cone penalties
    total_time = float(dt_arr.sum())

    return {
        'x': x, 'y': y, 'v': v, 'radius': radius, 'ds_arr': ds_arr, 'bounds': bounds,
        'right_lap_time': right_lap_time, 'left_lap_time': left_lap_time,
        'corrected_time': corrected_time, 'total_time': total_time, 'max_speed': float(v.max()),
        'aero_yaw_derate': float(derate_circle), 'body_slip_deg': float(np.rad2deg(beta_circle)),
    }

if __name__ == "__main__":
    # Imported here, not at the top, since YawPlane imports from this module
    sys.path.insert(0, str(RESOURCES_DIR / "VehicleDynamics"))
    from YawPlane import max_corner_speed_yaw_plane
    import matplotlib.pyplot as plt

    OUTPUT_DIR = Path(__file__).resolve().parent.parent / "Outputs" / "Skidpad"
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    def sweep_values(start, end, step):
        """Values from start to end (inclusive) in steps of step, rounded to clear float noise."""
        return np.round(np.arange(start, end + step / 1000.0, step), 6)

    # One single-driver, single-attempt run per car
    cars = {
        "SR_17": ("SR_17 (Single-Motor RWD)", SR_17),
        "SR_18": ("SR_18 (4-Motor AWD)", SR_18),
    }

    for key, (name, CarClass) in cars.items():
        car = CarClass()
        result = simulate_skidpad_run(car)

        print(f"--- FSAE Skidpad Simulation: {name} (single driver, single attempt) ---")
        print(f"Right Circle Lap Time (timed): {result['right_lap_time']:.3f} s")
        print(f"Left Circle Lap Time (timed):  {result['left_lap_time']:.3f} s")
        print(f"Corrected Time (D.10.4):       {result['corrected_time']:.3f} s")
        print(f"Total Time (entry to exit):    {result['total_time']:.3f} s")
        print(f"Max Speed Reached:             {result['max_speed'] * 3.6:.2f} km/h")
        print(f"Circle Body Slip Angle:        {result['body_slip_deg']:.2f} deg")
        print(f"Aero Yaw Derate on Circle:     {result['aero_yaw_derate']:.3f} "
              f"({(1.0 - result['aero_yaw_derate']) * 100.0:.1f}% downforce loss vs. straight-line running)")

        # Cross-check: circle speed from YawPlane's full force + yaw moment balance vs.
        # the static-weight split used above (both without aero yaw loss)
        yaw_v, yaw_result = max_corner_speed_yaw_plane(car, PATH_RADIUS)
        v_circle = max_corner_speed_bisect(car, PATH_RADIUS)
        delta_pct = (yaw_v * 3.6 - v_circle * 3.6) / (v_circle * 3.6) * 100.0
        print(f"Yaw-Plane Cross-Check: {yaw_v * 3.6:.2f} km/h vs. {v_circle * 3.6:.2f} km/h axle-split "
              f"steady-state circle speed ({delta_pct:+.2f}%)")
        if yaw_result:
            print(f"  at the limit: delta={np.rad2deg(yaw_result['delta']):.2f} deg, "
                  f"alpha_front={np.rad2deg(yaw_result['alpha_front']):.2f} deg, "
                  f"alpha_rear={np.rad2deg(yaw_result['alpha_rear']):.2f} deg")

        s = np.concatenate([[0.0], np.cumsum(result['ds_arr'])])
        fig, axs = plt.subplots(2, 1, figsize=(12, 8))
        axs[0].plot(s, result['v'] * 3.6)
        axs[0].set_xlabel("Distance Along Course [m]")
        axs[0].set_ylabel("Speed [km/h]")
        axs[0].set_title(f"{key} Skidpad Speed Trace")
        axs[0].grid(True)

        sc = axs[1].scatter(result['x'], result['y'], c=result['v'] * 3.6, cmap="turbo", s=6)
        axs[1].set_aspect("equal")
        axs[1].set_xlabel("x [m]")
        axs[1].set_ylabel("y [m]")
        axs[1].set_title(f"{key} Speed Around the Skidpad Course")
        fig.colorbar(sc, ax=axs[1], label="Speed [km/h]")

        plt.tight_layout()
        graph_path = OUTPUT_DIR / f"{key}_Skidpad_speed.png"
        plt.savefig(graph_path, dpi=150)
        plt.close(fig)
        print(f"Saved speed trace graph to {graph_path}")

        # Sweep: corrected time vs. tire pressure. Pressure is set when the car is built
        # (it sets the tire model and tire stiffness), so each point builds a new car.
        if RUN_TIRE_PRESSURE_SWEEP:
            psi_values = sweep_values(*TIRE_PRESSURE_RANGE)
            corrected_times_vs_psi = []
            for psi in psi_values:
                sweep_car = CarClass(tire_pressure_psi=psi)
                sweep_result = simulate_skidpad_run(sweep_car)
                corrected_times_vs_psi.append(sweep_result['corrected_time'])

            plt.figure(figsize=(8, 5))
            plt.plot(psi_values, corrected_times_vs_psi, marker='o', markersize=3)
            plt.axvline(8.0, color='gray', linestyle='--', linewidth=1,
                        label="8 psi: lowest pressure Pacejka_R20 has fitted data for")
            plt.xlabel("Tire Pressure [psi]")
            plt.ylabel("Corrected Time (s)")
            plt.title(f"{key} Skidpad Corrected Time vs. Tire Pressure")
            plt.legend()
            plt.grid(True)
            psi_graph_path = OUTPUT_DIR / f"{key}_Skidpad_psi_sweep.png"
            plt.savefig(psi_graph_path, dpi=150)
            plt.close()
            print(f"Saved psi sweep graph to {psi_graph_path}")
        print()
