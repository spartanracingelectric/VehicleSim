import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "Tire_R20"))

import numpy as np
from Pacejka_R20 import Pacejka_R20

class SR_17:
    def __init__(self, tire_pressure_psi=8.0):
        # Mass & Geometry Parameters
        self.mass = 272.11                          # Total mass with driver (kg)
        self.unsprung_mass_corner = 7.76643991      # Wheel/tire/upright/brake mass, per corner (kg)

        self.sprung_mass = self.mass - 4 * self.unsprung_mass_corner  # Chassis + driver, isolated by the springs (kg)

        self.rotational_inertia_factor_front = 1.02     # Spin-up correction for the undriven front wheels only -
                                                        # the rear driveline's rotational inertia is now modeled
                                                        # explicitly (see Drivetrain), so it's excluded here to
                                                        # avoid double-counting it
        
        self.wheelbase = 1.53924                    # Wheelbase L (m)
        self.cg_height = 0.312941529                # Center of gravity height h_cg (m)
        self.static_rear_bias = 0.509               # 50.9% static weight on rear axle
        self.wheel_radius = 0.203                   # Effective tire radius (m)

        # Aerodynamics Parameters
        self.air_density = 1.225                    # kg/m^3
        self.frontal_area = 1.16                    # Frontal area A (m^2)
        self.cd = 1.52                              # Drag coefficient
        self.cl = 4.08                              # Lift coefficient (downforce)
        self.aero_rear_bias = 0.5116                # Downforce split on rear axle

        # Tire Model - tire_forces()/tire_slip_ratio() below delegate to Pacejka_R20
        # (Tire_R20/Pacejka_R20.py), a Magic Formula pure-slip model fit directly to real
        # Calspan TTC Round 9 raw data for this tire's R20 compound (pooled across all 4
        # R20 carcass sizes tested that round - see Tire_R20/README.md). Unlike the Dugoff
        # model this replaced, it has a genuine peak followed by a falloff past peak slip
        # rather than only ever saturating. No flat tire_mu is kept here anymore - every
        # consumer (this class's own tire_forces(), and Skidpad.py's grip-circle check via
        # tire_peak_lateral()/tire_peak_longitudinal() below) now reads the real
        # load-dependent peak off the fitted curves instead.
        self.tire_pressure_psi = tire_pressure_psi  # Set per event via the constructor, e.g. SR_17(tire_pressure_psi=8.0)
                                         # - the TTC sweep tested 8/10/12/14 psi (see the Run Guide's pressure
                                         # schedule), and both Pacejka_R20 (below) and tire_vertical_spring_rate
                                         # interpolate their real fitted/measured values between those
                                         # setpoints, clamped to that 8-14 psi range. A constructor argument
                                         # rather than a plain post-construction attribute because both of
                                         # those derive from it once, here in __init__ - unlike this file's
                                         # other sweepable attributes (e.g. power_limit), reassigning
                                         # car.tire_pressure_psi after construction would NOT retroactively
                                         # update either of them.
        self.tires_camber_sensitive = False  # Per earlier TTC review: no meaningful camber sensitivity found
                                              # (Pacejka_R20 pools camber-sweep points as ordinary data rather
                                              # than fitting a separate camber term, consistent with this)

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
        # rate table" for this Hoosier R20 16x7.5-10, tire 4, post-test dynamic, at 0 deg camber, at each
        # of the 4 pressures the TTC sweep tested: 439.19/521.59/556.50/604.04 lbf/in at 8/10/12/14 psi
        # respectively (76,915/91,345/97,458/105,783 N/m) - linearly interpolated by tire_pressure_psi
        # above, clamped to that 8-14 psi range (no measured data outside it).
        self.tire_vertical_spring_rate = np.interp(self.tire_pressure_psi, [8.0, 10.0, 12.0, 14.0],
                                                     [76914.5, 91344.7, 97458.4, 105783.4])  # N/m
        self.track_width = 1.2065          # Track width, front & rear assumed equal (m)
        self.roll_stiffness_front = 0.45  # Fraction of lateral load transfer reacted by the front axle,
                                           # set by front/rear spring & anti-roll bar stiffness (0.5 = even split)
                                           # (used by Skidpad.py; not exercised in a straight-line accel run)

        # Drivetrain & Motor Parameters (single-speed RWD electric, EMRAX 228 MV LC)
        self.final_drive_ratio = 2.7      # Fixed single-stage final drive ratio
        self.drivetrain_efficiency = 0.94  # 96% motor efficiency * ~98% single-stage final drive
        self.throttle_request = 1.0      # Driver torque request, as a fraction of the motor's available torque
        self.speed_limit_rpm = 6500       # Motor controller cuts torque above this mechanical speed limit
        self.power_limit = 80000.0        # Electrical power limit from the inverter/accumulator (W), independent of the torque curve
        self.rotor_inertia = 0.018        # Bare motor rotor inertia, unreflected by the final drive ratio (kg*m^2)
        self.half_shaft_stiffness = 3000.0  # Rear half-shaft torsional stiffness (Nm/rad)
        self.half_shaft_damping = 15.0      # Rear half-shaft structural damping (Nm*s/rad)
        self.wheel_inertia_rear = 0.45      # Combined rear wheel+tire+hub rotational inertia, both sides (kg*m^2)

        # Slip Control (traction / launch control)
        self.enable_launch_control = True
        self.target_slip_ratio = 0.4     # Controller's target slip ratio (matches slip_ratio_peak by default)
        self.slip_control_gain = 12.0     # Proportional gain reining in torque once slip exceeds target

        # Motor Torque Curve: (RPM, Torque in Nm) - EMRAX 228 Medium Voltage, liquid cooled
        # Flat peak torque (220 Nm) up to the 124 kW peak-power point at 5500 RPM, then constant-power
        # roll-off to the 6500 RPM mechanical speed limit, where the controller cuts torque to zero.
        # (Previously this curve held 182.2 Nm flat past 6500 RPM indefinitely - not physical, since the
        # motor cannot exceed its rated mechanical speed limit at all.)
        self.rpm_points = np.array([0, 5500, 6500, 6500.001])
        self.torque_points = np.array([220.0, 220.0, 182.2, 0.0])

        self._tire_model = Pacejka_R20(wheel_radius=self.wheel_radius, psi=self.tire_pressure_psi)

    def get_motor_torque(self, rpm):
        """Linearly interpolates motor torque from the RPM curve (0 past the speed limit)."""
        return np.interp(rpm, self.rpm_points, self.torque_points, right=0.0)

    def tire_slip_ratio(self, wheel_omega, vx):
        """Longitudinal slip ratio kappa from wheel angular speed and vehicle speed -
        delegates to Pacejka_R20 (same convention as before: denominator is wheel surface
        speed, not vx)."""
        return self._tire_model.tire_slip_ratio(wheel_omega, vx)

    def tire_forces(self, fz, kappa, alpha=0.0):
        """Pacejka Magic Formula pure-slip tire model, fit directly to real Calspan TTC
        Round 9 data for this tire's R20 compound - see Tire_R20/README.md. Returns
        (Fx, Fy) in N for normal load fz (N), slip ratio kappa, and slip angle alpha
        (radians, default 0 for straight-line running - no steer, no lateral velocity).

        Unlike the Dugoff model this replaced, force genuinely peaks (around kappa~0.17-
        0.18 or alpha~11-12 deg) and falls off past it, rather than only ever saturating
        toward a friction-circle limit. Combined slip (both kappa and alpha nonzero) isn't
        independently fit - no such TTC sweep exists in this dataset - so it falls back to
        an approximate friction-circle scaling; see Pacejka_R20's docstring.
        """
        return self._tire_model.tire_forces(fz, kappa, alpha)

    def tire_peak_lateral(self, fz):
        """Peak achievable |Fy| (N) at this normal load - the real load-dependent grip
        curve, used by Skidpad.py's grip-circle check in place of a flat tire_mu*Fz."""
        return self._tire_model.peak_lateral(fz)

    def tire_peak_longitudinal(self, fz):
        """Peak achievable |Fx| (N) at this normal load - see tire_peak_lateral."""
        return self._tire_model.peak_longitudinal(fz)
