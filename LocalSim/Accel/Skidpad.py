import numpy as np
from SR_17 import SR_17
from SR_18 import SR_18

def solve_max_corner_speed(car, radius, v_search_max=60.0, dv=0.001):
    """Finds the maximum steady-state cornering speed for a given radius via a speed sweep.

    Lateral load transfer is split between axles by roll_stiffness_front and clamped at
    zero per corner, so an unloaded inside tire can't be compensated for by the outside
    tire - this is how the suspension's roll-stiffness distribution affects grip here.

    Two constraints must both hold at each candidate speed: the combined lateral +
    longitudinal force (drag + rolling resistance, needed to hold a constant speed)
    must fit within the tires' friction circle, and the drivetrain must be able to
    produce that longitudinal force at the motor RPM implied by reduction_ratio - this
    is what lets gearing matter for a grip-limited corner instead of only straight-line
    acceleration.
    """
    g = 9.81
    static_front_bias = 1.0 - car.static_rear_bias
    aero_front_bias = 1.0 - car.aero_rear_bias
    num_driven_motors = getattr(car, 'num_motors', 1)

    best_v = 0.0
    for v in np.arange(0.0, v_search_max, dv):
        a_lat = v**2 / radius
        f_down = 0.5 * car.air_density * car.cl * car.frontal_area * v**2

        f_z_front_total = car.mass * g * static_front_bias + f_down * aero_front_bias
        f_z_rear_total = car.mass * g * car.static_rear_bias + f_down * car.aero_rear_bias

        transfer_front = car.roll_stiffness_front * car.mass * a_lat * car.cg_height / car.track_width
        transfer_rear = (1.0 - car.roll_stiffness_front) * car.mass * a_lat * car.cg_height / car.track_width

        f_z_front_outer = max(0.0, f_z_front_total / 2.0 + transfer_front)
        f_z_front_inner = max(0.0, f_z_front_total / 2.0 - transfer_front)
        f_z_rear_outer = max(0.0, f_z_rear_total / 2.0 + transfer_rear)
        f_z_rear_inner = max(0.0, f_z_rear_total / 2.0 - transfer_rear)

        f_grip_available = car.tire_mu * (f_z_front_outer + f_z_front_inner + f_z_rear_outer + f_z_rear_inner)

        # Friction-circle check: lateral grip use plus the longitudinal force needed
        # to hold a constant speed (overcoming drag + rolling resistance) combined
        f_lat_required = car.mass * a_lat
        f_drag = 0.5 * car.air_density * car.cd * car.frontal_area * v**2
        f_rr = car.crr * car.mass * g
        f_long_required = f_drag + f_rr
        f_combined_required = np.hypot(f_lat_required, f_long_required)

        # Drivetrain check: can the motor(s), at the RPM implied by this ratio, actually
        # deliver that longitudinal force?
        wheel_rad_s = v / car.wheel_radius
        motor_rpm = (wheel_rad_s * 60.0) / (2.0 * np.pi) * car.reduction_ratio
        wheel_torque = car.get_motor_torque(motor_rpm) * car.reduction_ratio * car.drivetrain_efficiency
        f_long_achievable = num_driven_motors * wheel_torque / car.wheel_radius

        if f_combined_required > f_grip_available or f_long_required > f_long_achievable:
            return best_v
        best_v = v

    return best_v

def solve_crossover_transition_time(car):
    """Estimates the time cost of the figure-8's single crossover, where the car reverses
    from 2 laps of steady-state cornering in one direction to 2 laps in the other.

    Lateral load transfer can't reverse instantly - each axle's outer wheel has to unload
    and the inner wheel load up in the other direction, paced by that axle's actual spring/
    damper, same physics (and the same k_eff/damping formulas) as the longitudinal weight-
    transfer suspension model in Acceleration_RWD.py's simulate_75m_accel, just reacting a
    lateral load-transfer target instead of a longitudinal one. Modeled as each axle's
    lateral load transfer being a 2nd-order spring-damper system whose forcing target steps
    from +transfer_target (steady right-hand circle) to -transfer_target (steady left-hand
    circle); the transition time is the slower axle's standard 2% step-response settling
    time, 4/(zeta*omega_n), added on top of the 4 steady-state laps. damping_ratio_front/rear
    are already defined as the spring/damper's damping fraction of critical (zeta) - see
    Acceleration_RWD.py's c_front/c_rear.

    This still doesn't model the crossover's path geometry/distance (undimensioned on the
    layout, as before) - only the extra time the roll dynamics cost while reversing.
    """
    static_front_bias = 1.0 - car.static_rear_bias

    k_wheel_front = car.spring_rate_front * car.motion_ratio_front**2
    k_eff_front_axle = 2.0 * (k_wheel_front * car.tire_vertical_spring_rate) / (k_wheel_front + car.tire_vertical_spring_rate)
    m_front_sprung = car.sprung_mass * static_front_bias
    omega_n_front = np.sqrt(k_eff_front_axle / m_front_sprung)

    k_wheel_rear = car.spring_rate_rear * car.motion_ratio_rear**2
    k_eff_rear_axle = 2.0 * (k_wheel_rear * car.tire_vertical_spring_rate) / (k_wheel_rear + car.tire_vertical_spring_rate)
    m_rear_sprung = car.sprung_mass * car.static_rear_bias
    omega_n_rear = np.sqrt(k_eff_rear_axle / m_rear_sprung)

    t_settle_front = 4.0 / (car.damping_ratio_front * omega_n_front)
    t_settle_rear = 4.0 / (car.damping_ratio_rear * omega_n_rear)

    return max(t_settle_front, t_settle_rear)

def simulate_skidpad(car, control_line_diameter=15.25):
    """Simulates the FSAE figure-8 skidpad: 2 CW laps of the right circle, then 2 CCW
    laps of the left circle, each driven at its maximum steady-state cornering speed,
    plus the transient at the single crossover between the two circles (see
    solve_crossover_transition_time) where lateral load transfer has to reverse.

    The course's entry/exit straights and the crossover's path geometry/distance aren't
    dimensioned on the layout, so those still aren't modeled - only the crossover's roll-
    dynamics time cost is.
    """
    radius = control_line_diameter / 2.0
    v_max = solve_max_corner_speed(car, radius)

    lap_distance = 2.0 * np.pi * radius
    lap_time = lap_distance / v_max
    transition_time = solve_crossover_transition_time(car)
    total_time = 4 * lap_time + transition_time  # 2 laps right + 2 laps left, both at the same speed by symmetry,
                                                   # plus the one crossover transition between them

    return total_time, lap_time, v_max, transition_time

# Run default simulation for both cars
cars = {
    "SR_17 (Single-Motor RWD)": SR_17(),
    "SR_18 (4-Motor AWD)": SR_18(),
}

for name, car in cars.items():
    elapsed_time, lap_time, max_speed, transition_time = simulate_skidpad(car)

    print(f"--- FSAE Skidpad Simulation: {name} (Figure-8: 2 CW + 2 CCW laps) ---")
    print(f"Max Cornering Speed:   {max_speed * 3.6:.2f} km/h")
    print(f"Time per Lap:          {lap_time:.3f} s")
    print(f"Crossover Transition:  {transition_time:.3f} s")
    print(f"Total Time (4 laps + crossover): {elapsed_time:.3f} s")
    print()
