import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "Tire_R20"))

import numpy as np
from Pacejka_R20 import Pacejka_R20

class SR_18:
    def __init__(self, tire_pressure_psi=12.0):
        # Mass & Geometry Parameters
        self.mass = 288.0                           # Total mass with driver (kg) - AMK DD5 motors are light (3.55 kg each, 14.2 kg total),
                                                    # but 4 corners each need their own inverter/wiring/final drive hardware vs SR_17's single unit
        self.unsprung_mass_corner = 16      # Wheel/tire/upright/brake mass, per corner (kg) - same corner hardware as SR_17;
                                                    # not re-measured for the in-wheel AMK motor's added unsprung mass
        
        self.sprung_mass = self.mass - 4 * self.unsprung_mass_corner    # Chassis + driver, isolated by the springs (kg)

        self.rotational_inertia_factor = 1.07       # Rotational mass correction factor (~7%, more rotating driveline mass with 4 motors)
        self.wheelbase = 1.53924                    # Wheelbase L (m)
        self.cg_height = 0.32512                    # Center of gravity height h_cg (m)
        self.static_rear_bias = 0.50                # Near 50/50 static weight split (front motors added for AWD)
        self.wheel_radius = 0.203                   # Effective tire radius (m)

        # Aerodynamics Parameters
        self.air_density = 1.225                    # kg/m^3
        self.frontal_area = 1.16                    # Frontal area A (m^2)
        self.cd = 1.52                              # Drag coefficient
        self.cl = 4.08                              # Lift coefficient (downforce)
        self.aero_rear_bias = 0.5116                # Downforce split on rear axle

        # Tire Model
        # 
        # Same as SR_17.py (same physical tire, front & rear): tire_forces()/
        # tire_slip_ratio() below delegate to Pacejka_R20 (Tire_R20/Pacejka_R20.py), a Magic
        # Formula pure-slip model fit directly to real Calspan TTC Round 9 raw data for this
        # R20 compound (pooled across all 4 R20 carcass sizes tested that round - see
        # Tire_R20/README.md). Unlike the Dugoff model this replaced, it has a genuine peak
        # followed by a falloff past peak slip rather than only ever saturating. No flat
        # tire_mu is kept here anymore - every consumer (this class's own tire_forces(), and
        # Skidpad.py's grip-circle check via tire_peak_lateral()/tire_peak_longitudinal()
        # below) now reads the real load-dependent peak off the fitted curves instead.
        self.tire_pressure_psi = tire_pressure_psi  # Set per event via the constructor, e.g. SR_18(tire_pressure_psi=8.0)
                                         # - the TTC sweep tested 8/10/12/14 psi (see the Run Guide's pressure
                                         # schedule), and both Pacejka_R20 (below) and tire_vertical_spring_rate
                                         # interpolate their real fitted/measured values between those
                                         # setpoints, clamped to that 8-14 psi range. A constructor argument
                                         # rather than a plain post-construction attribute because both of
                                         # those derive from it once, here in __init__ - unlike this file's
                                         # other sweepable attributes (e.g. power_limit), reassigning
                                         # car.tire_pressure_psi after construction would NOT retroactively
                                         # update either of them.
        self.tires_camber_sensitive = False         # Per earlier TTC review: no meaningful camber sensitivity found
                                                    # (Pacejka_R20 pools camber-sweep points as ordinary data rather
                                                    # than fitting a separate camber term, consistent with this)
        
        self.crr = 0.015                            # Rolling resistance coefficient

        # Suspension Parameters - Front (same spring/damper hardware as SR_17; not re-tuned for AWD)
        self.spring_rate_front = 70050.73           # Spring rate at the spring, front (N/m)
        self.motion_ratio_front = 0.85              # Wheel travel / spring travel, front
        self.damping_ratio_front = 0.76             # Damper coefficient as a fraction of critical damping, front
        self.anti_lift_front = 0.11                 # Fraction of front load transfer (loss) reacted through suspension links
        self.anti_dive_front = 0.33                 # Fraction of front load transfer (gain) reacted through links while decelerating
                                                    # (same geometry as SR_17; not re-tuned for AWD)

        # Suspension Parameters - Rear
        self.spring_rate_rear = 80208.09            # Spring rate at the spring, rear (N/m)
        self.motion_ratio_rear = 0.87               # Wheel travel / spring travel, rear
        self.damping_ratio_rear = 0.68              # Damper coefficient as a fraction of critical damping, rear
        self.anti_squat_rear = 0.44                 # Fraction of rear load transfer reacted through suspension links
        self.anti_lift_rear = 0.11                  # Same idea, but for rear load transfer during deceleration
                                                    # (rear unloading under braking/coast/regen instead of squatting;
                                                    # same geometry as SR_17, not re-tuned for AWD)

        # Suspension Parameters - shared
        # Same tire as SR_17 (see SR_17.py for the Calspan TTC "Pre+post spring rate table"
        # source), interpolated by tire_pressure_psi across the same 4 tested setpoints:
        # 439.19/521.59/556.50/604.04 lbf/in at 8/10/12/14 psi (76,915/91,345/97,458/105,783 N/m)
        self.tire_vertical_spring_rate = np.interp(self.tire_pressure_psi, [8.0, 10.0, 12.0, 14.0],
                                                     [76914.5, 91344.7, 97458.4, 105783.4])  # N/m
        self.track_width = 1.2065          # Track width, front & rear assumed equal (m)
        self.roll_stiffness_front = 0.45  # Fraction of lateral load transfer reacted by the front axle,
                                           # set by front/rear spring & anti-roll bar stiffness (0.5 = even split)

        # Drivetrain & Motor Parameters (4-motor AWD, one AMK DD5-14-10-POW per wheel)
        self.num_motors = 4             # One motor per wheel
        self.final_drive_ratio = 11.81      # Fixed single-stage final drive ratio (per motor)
        self.drivetrain_efficiency = 0.94  # Liquid-cooled synchronous motor + single-stage final drive
        self.torque_split_front = 0.50  # Fraction of commanded drive torque sent to the front axle (rear gets the rest);
                                         # all 4 motors are identical hardware, so this is a controller torque-vectoring
                                         # bias, not a difference in motor capability
        self.throttle_request = 1.0     # Driver torque request, as a fraction of each motor's available torque
        self.speed_limit_rpm = 20000    # Motor's mechanical speed limit "Nmax" (AMK DD5-14-10-POW datasheet) -
                                         # separate from the 18617 RPM theoretical no-load point already built
                                         # into the torque curve below
        self.power_limit = 80000.0      # Electrical power limit from the inverter/accumulator (W), shared across
                                         # all 4 motors - same accumulator as SR_17 (rules-limited, not motor-count-
                                         # dependent), independent of any single motor's torque curve
        self.rotor_inertia = 0.000274    # Bare rotor inertia, per motor, unreflected by the final drive ratio (kg*m^2) -
                                          # AMK DD5-14-10-POW datasheet: J = 2.74 kgcm^2
        self.half_shaft_stiffness = 3000.0  # Axle-level effective driveline torsional stiffness, motor/final drive
                                             # output to wheel hub (Nm/rad) - engineering estimate carried over
                                             # directly from SR_17's characterized rear axle value (same number, same
                                             # units); not vendor-sourced for the in-hub AMK driveline. Used per axle
                                             # in Acceleration_AWD.py's 2-mass torsional model (one twist state per
                                             # axle, assuming the axle's 2 wheels stay symmetric under straight-line
                                             # running), not doubled for the 2 wheels it spans
        self.half_shaft_damping = 15.0      # Axle-level effective driveline torsional damping (Nm*s/rad) - same
                                             # caveat and usage as half_shaft_stiffness above
        self.wheel_inertia_corner = 0.225   # Combined wheel+tire+hub rotational inertia, per single corner (kg*m^2) -
                                             # half of SR_17's wheel_inertia_rear (which lumped both rear wheels into
                                             # one node); all 4 corners share the same tire/wheel hardware here since
                                             # every wheel has its own independent motor rather than a shared axle.
                                             # Acceleration_AWD.py doubles this back to an axle-level wheel inertia
                                             # (2 wheels per axle) when building its per-axle torsional model

        # Slip Control (traction / launch control)
        self.enable_launch_control = True
        self.target_slip_ratio = 0.10     # Controller's target slip ratio (matches slip_ratio_peak by default)
        self.slip_control_gain = 12.0     # Proportional gain reining in torque once slip exceeds target

        # Motor Torque Curve: (RPM, Torque in Nm) - AMK DD5-14-10-POW, liquid cooled
        # Flat peak torque (Mmax = 21 Nm) up to the 12000 RPM rated speed, then constant-power
        # roll-off (peak power extrapolated as Mmax/Mn * Pn = 21/9.8 * 12.3 kW ~= 26.4 kW, scaling
        # the datasheet's rated 12.3 kW/9.8 Nm point by the peak/rated torque ratio) down to zero
        # torque at the 18617 RPM theoretical no-load speed.
        # This curve is per-motor; total torque at a wheel is this value (4 independent motors, one per wheel).
        self.rpm_points = np.array([0, 12000, 15000, 18617])
        self.torque_points = np.array([21.0, 21.0, 16.8, 0.0])

        self._tire_model = Pacejka_R20(wheel_radius=self.wheel_radius, psi=self.tire_pressure_psi)

    def get_motor_torque(self, rpm):
        """Linearly interpolates per-motor torque from the RPM curve (0 past the speed limit)."""
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
