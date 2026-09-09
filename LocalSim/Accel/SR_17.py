import numpy as np

class SR_17:
    def __init__(self):
        # Mass & Geometry Parameters
        self.mass = 272.11              # Total mass with driver (kg)
        self.unsprung_mass_corner = 7.76643991  # Wheel/tire/upright/brake mass, per corner (kg)
        self.sprung_mass = self.mass - 4 * self.unsprung_mass_corner  # Chassis + driver, isolated by the springs (kg)
        self.rotational_inertia_factor_front = 1.02  # Spin-up correction for the undriven front wheels only -
                                                       # the rear driveline's rotational inertia is now modeled
                                                       # explicitly (see Drivetrain), so it's excluded here to
                                                       # avoid double-counting it
        self.wheelbase = 1.53924          # Wheelbase L (m)
        self.cg_height = 0.312941529           # Center of gravity height h_cg (m)
        self.static_rear_bias = 0.509    # 50.9% static weight on rear axle
        self.wheel_radius = 0.203       # Effective tire radius (m)

        # Aerodynamics Parameters
        self.air_density = 1.225        # kg/m^3
        self.frontal_area = 1.16         # Frontal area A (m^2)
        self.cd = 1.52                  # Drag coefficient
        self.cl = 4.08                  # Lift coefficient (downforce)
        self.aero_rear_bias = 0.5116      # Downforce split on rear axle

        # Tire Model - Dugoff-style combined-slip model, ported from the team's TireConfig2.py /
        # tiretesting2.py characterization for this tire (16" R20, extrapolated from TTC data).
        # Cs, Ca, and D (the friction circle radius) all scale with the dynamic normal load Fz,
        # so grip responds to the suspension model's dynamic Fz instead of using a fixed mu.
        self.tire_mu = 1.5              # Tire/road friction coefficient (tiretesting2.py's extrapolated value)
        self.tire_c_long = 25.691       # Normalized longitudinal slip stiffness: Cs = tire_c_long * Fz
        self.tire_c_lat = 33.959        # Normalized cornering stiffness: Ca = tire_c_lat * Fz
                                         # (not exercised by a straight-line run; kept for a future combined-slip event)
        self.tires_camber_sensitive = False  # Per earlier TTC review: no meaningful camber sensitivity found
                                              # (not modeled here either way - this tire model has no camber input)

        self.crr = 0.015                # Rolling resistance coefficient

        # Suspension Parameters - Front
        self.spring_rate_front = 70050.73   # Spring rate at the spring, front (N/m)
        self.motion_ratio_front = 0.85     # Wheel travel / spring travel, front
        self.damping_ratio_front = 0.76    # Damper coefficient as a fraction of critical damping, front
        self.anti_lift_front = 0.11        # Fraction of front load transfer (loss) reacted through links while accelerating
        self.anti_dive_front = 0.33        # Fraction of front load transfer (gain) reacted through links while decelerating

        # Suspension Parameters - Rear
        self.spring_rate_rear = 80208.09    # Spring rate at the spring, rear (N/m)
        self.motion_ratio_rear = 0.87      # Wheel travel / spring travel, rear
        self.damping_ratio_rear = 0.68     # Damper coefficient as a fraction of critical damping, rear
        self.anti_squat_rear = 0.44     # Fraction of rear load transfer reacted through suspension links
                                         # instead of showing up as extra dynamic tire load (0 = none, 1 = full)
        self.anti_lift_rear = 0.11      # Same idea, but for rear load transfer during deceleration
                                         # (rear unloading under braking/coast/regen instead of squatting)

        # Suspension Parameters - shared
        # Tire vertical spring rate, kt: measured (not assumed) - Calspan TTC Round 9 "Pre+post spring
        # rate table" for this Hoosier R20 16x7.5-10, tire 4, post-test dynamic, 12 psi (typical running
        # pressure) at 0 deg camber: 556.5 lbf/in = 97,458 N/m. The same table shows this is pressure-
        # sensitive (439 lbf/in at 8 psi up to 604 lbf/in at 14 psi) - rerun the conversion if actual
        # running pressure differs from 12 psi.
        self.tire_vertical_spring_rate = 97458.0  # N/m
        self.track_width = 1.2065          # Track width, front & rear assumed equal (m)
        self.roll_stiffness_front = 0.45  # Fraction of lateral load transfer reacted by the front axle,
                                           # set by front/rear spring & anti-roll bar stiffness (0.5 = even split)
                                           # (used by Skidpad.py; not exercised in a straight-line accel run)

        # Drivetrain & Motor Parameters (single-speed RWD electric, EMRAX 228 MV LC)
        self.reduction_ratio = 3.5      # Fixed single-stage reduction ratio
        self.drivetrain_efficiency = 0.94  # 96% motor efficiency * ~98% single-stage reduction
        self.throttle_request = 1.0      # Driver torque request, as a fraction of the motor's available torque
        self.speed_limit_rpm = 6500       # Motor controller cuts torque above this mechanical speed limit
        self.power_limit = 80000.0        # Electrical power limit from the inverter/accumulator (W), independent of the torque curve
        self.rotor_inertia = 0.018        # Bare motor rotor inertia, unreflected by the reduction ratio (kg*m^2)
        self.half_shaft_stiffness = 3000.0  # Rear half-shaft torsional stiffness (Nm/rad)
        self.half_shaft_damping = 15.0      # Rear half-shaft structural damping (Nm*s/rad)
        self.wheel_inertia_rear = 0.45      # Combined rear wheel+tire+hub rotational inertia, both sides (kg*m^2)

        # Slip Control (traction / launch control)
        self.enable_launch_control = False
        self.target_slip_ratio = 0.10     # Controller's target slip ratio (matches slip_ratio_peak by default)
        self.slip_control_gain = 12.0     # Proportional gain reining in torque once slip exceeds target

        # Motor Torque Curve: (RPM, Torque in Nm) - EMRAX 228 Medium Voltage, liquid cooled
        # Flat peak torque (220 Nm) up to the 124 kW peak-power point at 5500 RPM, then constant-power
        # roll-off to the 6500 RPM mechanical speed limit, where the controller cuts torque to zero.
        # (Previously this curve held 182.2 Nm flat past 6500 RPM indefinitely - not physical, since the
        # motor cannot exceed its rated mechanical speed limit at all.)
        self.rpm_points = np.array([0, 5500, 6500, 6500.001])
        self.torque_points = np.array([220.0, 220.0, 182.2, 0.0])

    def get_motor_torque(self, rpm):
        """Linearly interpolates motor torque from the RPM curve (0 past the speed limit)."""
        return np.interp(rpm, self.rpm_points, self.torque_points, right=0.0)

    def tire_slip_ratio(self, wheel_omega, vx):
        """Longitudinal slip ratio kappa from wheel angular speed and vehicle speed,
        matching TireConfig2.calculateslip (denominator is wheel surface speed, not vx)."""
        wheel_speed = wheel_omega * self.wheel_radius
        denom = wheel_speed if abs(wheel_speed) > 1e-6 else 1e-6
        kappa = (wheel_speed - vx) / denom
        return max(-0.999, min(0.999, kappa))

    def tire_forces(self, fz, kappa, alpha=0.0):
        """Dugoff-style combined-slip tire model, ported from the team's TireConfig2.py.
        Returns (Fx, Fy) in N for normal load fz (N), slip ratio kappa, and slip angle
        alpha (radians, default 0 for straight-line running - no steer, no lateral velocity).

        Note this model's saturation function is monotonically non-decreasing in combined
        slip (it approaches the friction circle limit tire_mu*fz but never falls back off
        it) - unlike a "wheelspin costs grip past peak" curve, running past peak slip here
        just wastes time building torque through the driveline, it doesn't reduce force.
        """
        if fz <= 0.0:
            return 0.0, 0.0
        Cs = self.tire_c_long * fz
        Ca = self.tire_c_lat * fz
        D = self.tire_mu * fz

        kappahat = (kappa * (Cs / D)) / (1.0 - kappa)
        alphahat = (np.tan(alpha) * (Ca / D)) / (1.0 - kappa)
        shat = np.hypot(kappahat, alphahat)

        if shat == 0.0:
            return 0.0, 0.0
        elif shat < 0.5:
            f_res = D * shat
        else:
            f_res = D * (1.0 - 1.0 / (4.0 * shat))

        fx = (kappahat / shat) * f_res
        fy = (alphahat / shat) * f_res
        return fx, fy
