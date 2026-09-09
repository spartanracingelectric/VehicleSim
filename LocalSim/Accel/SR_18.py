import numpy as np

class SR_18:
    def __init__(self):
        # Mass & Geometry Parameters
        self.mass = 288.0                       # Total mass with driver (kg) - AMK DD5 motors are light (3.55 kg each, 14.2 kg total),
                                                 # but 4 corners each need their own inverter/wiring/reduction hardware vs SR_17's single unit
        self.unsprung_mass_corner = 7.76643991  # Wheel/tire/upright/brake mass, per corner (kg) - same corner hardware as SR_17;
                                                 # not re-measured for the in-wheel AMK motor's added unsprung mass
        self.sprung_mass = self.mass - 4 * self.unsprung_mass_corner  # Chassis + driver, isolated by the springs (kg)
        self.rotational_inertia_factor = 1.07   # Rotational mass correction factor (~7%, more rotating driveline mass with 4 motors)
        self.wheelbase = 1.53924                # Wheelbase L (m)
        self.cg_height = 0.32512                # Center of gravity height h_cg (m)
        self.static_rear_bias = 0.50            # Near 50/50 static weight split (front motors added for AWD)
        self.wheel_radius = 0.203               # Effective tire radius (m)

        # Aerodynamics Parameters
        self.air_density = 1.225        # kg/m^3
        self.frontal_area = 1.16         # Frontal area A (m^2)
        self.cd = 1.52                  # Drag coefficient
        self.cl = 4.08                  # Lift coefficient (downforce)
        self.aero_rear_bias = 0.5116      # Downforce split on rear axle

        # Tire Model - Dugoff-style combined-slip model, same as SR_17.py (same physical tire:
        # 16" R20, ported from the team's TireConfig2.py / tiretesting2.py characterization).
        # Cs, Ca, and D (the friction circle radius) all scale with the dynamic normal load Fz,
        # so grip responds to the suspension model's dynamic Fz instead of using a fixed mu.
        self.tire_mu = 1.5              # Tire/road friction coefficient (same tires front & rear)
        self.tire_c_long = 25.691       # Normalized longitudinal slip stiffness: Cs = tire_c_long * Fz
        self.tire_c_lat = 33.959        # Normalized cornering stiffness: Ca = tire_c_lat * Fz
                                         # (not exercised by a straight-line run; kept for a future combined-slip event)
        self.crr = 0.015                # Rolling resistance coefficient

        # Suspension Parameters - Front (same spring/damper hardware as SR_17; not re-tuned for AWD)
        self.spring_rate_front = 70050.73   # Spring rate at the spring, front (N/m)
        self.motion_ratio_front = 0.85     # Wheel travel / spring travel, front
        self.damping_ratio_front = 0.76    # Damper coefficient as a fraction of critical damping, front
        self.anti_lift_front = 0.11        # Fraction of front load transfer (loss) reacted through suspension links

        # Suspension Parameters - Rear
        self.spring_rate_rear = 80208.09    # Spring rate at the spring, rear (N/m)
        self.motion_ratio_rear = 0.87      # Wheel travel / spring travel, rear
        self.damping_ratio_rear = 0.68     # Damper coefficient as a fraction of critical damping, rear
        self.anti_squat_rear = 0.44     # Fraction of rear load transfer reacted through suspension links

        # Suspension Parameters - shared
        self.tire_vertical_spring_rate = 97458.0  # N/m - same tire as SR_17 (see SR_17.py for the Calspan TTC source)
        self.track_width = 1.2065          # Track width, front & rear assumed equal (m)
        self.roll_stiffness_front = 0.45  # Fraction of lateral load transfer reacted by the front axle,
                                           # set by front/rear spring & anti-roll bar stiffness (0.5 = even split)

        # Drivetrain & Motor Parameters (4-motor AWD, one AMK DD5-14-10-POW per wheel)
        self.num_motors = 4             # One motor per wheel
        self.reduction_ratio = 11.81      # Fixed single-stage reduction ratio (per motor)
        self.drivetrain_efficiency = 0.94  # Liquid-cooled synchronous motor + single-stage reduction
        self.torque_split_front = 0.50  # Fraction of commanded drive torque sent to the front axle (rear gets the rest);
                                         # all 4 motors are identical hardware, so this is a controller torque-vectoring
                                         # bias, not a difference in motor capability

        # Motor Torque Curve: (RPM, Torque in Nm) - AMK DD5-14-10-POW, liquid cooled
        # Flat peak torque (Mmax = 21 Nm) up to the 12000 RPM rated speed, then constant-power
        # roll-off (peak power extrapolated as Mmax/Mn * Pn = 21/9.8 * 12.3 kW ~= 26.4 kW, scaling
        # the datasheet's rated 12.3 kW/9.8 Nm point by the peak/rated torque ratio) down to zero
        # torque at the 18617 RPM theoretical no-load speed.
        # This curve is per-motor; total torque at a wheel is this value (4 independent motors, one per wheel).
        self.rpm_points = np.array([0, 12000, 15000, 18617])
        self.torque_points = np.array([21.0, 21.0, 16.8, 0.0])

    def get_motor_torque(self, rpm):
        """Linearly interpolates per-motor torque from the RPM curve."""
        return np.interp(rpm, self.rpm_points, self.torque_points)

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
