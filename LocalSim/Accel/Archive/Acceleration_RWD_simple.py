import numpy as np
from SR_17_simple import SR_17_simple

def simulate_75m_accel(car, target_distance=75.0, dt=0.0005):
    """Simulates acceleration over the given distance using Euler integration.

    Same dynamic suspension model as Acceleration_RWD.py (each axle is a real
    spring/damper reacting the weight-transfer force), but the tire and drivetrain
    are simplified:
      - Traction is a plain mu*Fz cap (min of what the motor can drive vs. what the
        tire can hold), not a slip-ratio curve.
      - The driveline is treated as rigid: motor RPM comes directly from vehicle
        speed via the fixed reduction ratio, with no separate wheel-speed state or
        half-shaft compliance (those existed in the full model only to support the
        slip calculation, so they're dropped along with the tire model).
    """
    g = 9.81
    static_front_bias = 1.0 - car.static_rear_bias
    aero_front_bias = 1.0 - car.aero_rear_bias

    # Effective per-axle suspension properties (spring, in series with tire compliance,
    # reacting the sprung mass on that axle)
    k_wheel_front = car.spring_rate_front * car.motion_ratio_front**2
    k_eff_front_axle = 2.0 * (k_wheel_front * car.tire_vertical_spring_rate) / (k_wheel_front + car.tire_vertical_spring_rate)
    m_front_sprung = car.sprung_mass * static_front_bias
    c_front = car.damping_ratio_front * 2.0 * np.sqrt(k_eff_front_axle * m_front_sprung)

    k_wheel_rear = car.spring_rate_rear * car.motion_ratio_rear**2
    k_eff_rear_axle = 2.0 * (k_wheel_rear * car.tire_vertical_spring_rate) / (k_wheel_rear + car.tire_vertical_spring_rate)
    m_rear_sprung = car.sprung_mass * car.static_rear_bias
    c_rear = car.damping_ratio_rear * 2.0 * np.sqrt(k_eff_rear_axle * m_rear_sprung)

    # Initial conditions (small initial velocity to account for starting 0.3m back)
    t, x, v, a = 0.0, 0.0, 2.10277778, 0.0
    z_f, zdot_f = 0.0, 0.0
    z_r, zdot_r = 0.0, 0.0

    log = {'t': [], 'x': [], 'v': [], 'a': [], 'rpm': [], 'fz_rear': [], 'fz_front': []}

    while x < target_distance:
        # Aero forces
        f_drag = 0.5 * car.air_density * car.cd * car.frontal_area * (v**2)
        f_down = 0.5 * car.air_density * car.cl * car.frontal_area * (v**2)

        # Weight-transfer forcing target for each axle's suspension (reduced by anti-squat/
        # anti-lift/anti-dive, whichever applies given the sign of the current acceleration)
        rear_coeff = car.anti_squat_rear if a >= 0.0 else car.anti_lift_rear
        front_coeff = car.anti_lift_front if a >= 0.0 else car.anti_dive_front
        transfer_base = (car.mass * a * car.cg_height) / car.wheelbase
        f_transfer_rear_target = transfer_base * (1.0 - rear_coeff)
        f_transfer_front_target = -transfer_base * (1.0 - front_coeff)

        # Suspension dynamics: each axle is a real spring/damper responding to that target
        zdotdot_r = (f_transfer_rear_target - k_eff_rear_axle * z_r - c_rear * zdot_r) / m_rear_sprung
        zdotdot_f = (f_transfer_front_target - k_eff_front_axle * z_f - c_front * zdot_f) / m_front_sprung

        f_z_dynamic_rear = k_eff_rear_axle * z_r + c_rear * zdot_r
        f_z_dynamic_front = k_eff_front_axle * z_f + c_front * zdot_f

        f_z_static_rear = car.mass * g * car.static_rear_bias
        f_z_static_front = car.mass * g * static_front_bias
        f_z_aero_rear = f_down * car.aero_rear_bias
        f_z_aero_front = f_down * aero_front_bias

        f_z_rear = max(0.0, f_z_static_rear + f_z_dynamic_rear + f_z_aero_rear)
        f_z_front = max(0.0, f_z_static_front + f_z_dynamic_front + f_z_aero_front)

        # Maximum available traction limit at rear wheels (simple constant-mu cap)
        f_traction_max = car.tire_mu * f_z_rear

        # Motor RPM (rigid driveline, single fixed reduction ratio, no slip)
        wheel_rad_s = v / car.wheel_radius
        motor_rpm = (wheel_rad_s * 60.0) / (2.0 * np.pi) * car.reduction_ratio

        # Drive force from motor torque curve
        torque_avail = car.get_motor_torque(motor_rpm) * car.throttle_request

        # Electrical power limit (inverter/accumulator), independent of the torque curve
        motor_omega = wheel_rad_s * car.reduction_ratio
        power_draw = torque_avail * motor_omega
        if power_draw > car.power_limit and motor_omega > 1e-6:
            torque_avail = car.power_limit / motor_omega

        wheel_torque = torque_avail * car.reduction_ratio * car.drivetrain_efficiency
        f_drive = wheel_torque / car.wheel_radius

        # Tractive force capped by tire friction limit
        f_x = min(f_drive, f_traction_max)

        # Rolling Resistance
        f_rr = car.crr * car.mass * g

        # Acceleration calculation (with effective rotational inertia)
        m_eff = car.mass * car.rotational_inertia_factor
        a = (f_x - f_drag - f_rr) / m_eff

        # Integration step
        v += a * dt
        x += v * dt
        t += dt

        zdot_r += zdotdot_r * dt
        z_r += zdot_r * dt
        zdot_f += zdotdot_f * dt
        z_f += zdot_f * dt

        # Logging
        log['t'].append(t)
        log['x'].append(x)
        log['v'].append(v * 3.6) # Store speed in km/h
        log['a'].append(a / g)   # Store acceleration in g's
        log['rpm'].append(motor_rpm)
        log['fz_rear'].append(f_z_rear)
        log['fz_front'].append(f_z_front)

    return t, log

# Run default simulation
fsae_car = SR_17_simple()
elapsed_time, sim_log = simulate_75m_accel(fsae_car)

print(f"--- FSAE 75m Acceleration Simulation (Single-Motor RWD Electric, simple tire model) ---")
print(f"Elapsed Time: {elapsed_time:.3f} s")
print(f"Trap Speed:   {sim_log['v'][-1]:.2f} km/h")
print(f"Max Accel:    {max(sim_log['a']):.2f} g")
