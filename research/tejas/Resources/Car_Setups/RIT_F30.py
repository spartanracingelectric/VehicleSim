"""RIT_F30: RIT's single-motor RWD electric car, set up with the same attributes and methods
as SR_17 so the sims can run it in SR_17's place.

Source: RIT_2023.xlsx (an OpenLAP vehicle file, sheets "Info" and "Torque Curve"). Values
from the sheet are marked [sheet]. The sheet has no CG height, track width, suspension,
inertia, half-shaft, launch-control or power-limit data, so those are SR_17's values,
marked [ASSUMED = SR_17]. Replace them when RIT's numbers are known.

Sheet values NOT used, because these sims model them differently:
  - Tyres: OpenLAP's friction model (mu 1.64, load sensitivity 0.0001/N, cornering stiffness
    800/1000 N/deg). The sims use a fitted Hoosier 16.0x7.5-10 tire model instead: this car's
    default is LC0_V2_Tire_Model (SR_17's is the R20); the Accel and Steady_State_Grip sims can
    still pick either. The sheet's 203.2 mm tire radius is a 16" tire, consistent with these.
  - Brakes, steering rack ratio: not read by the Accel, QSS or Steady_State_Grip sims.
  - Thermal efficiency / fuel heating value: OpenLAP's engine-energy fields (ICE defaults).
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "Tire_R20"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "Tire_LC0"))

import numpy as np
from LC0_V2_Tire_Model import LC0_V2_Tire_Model


class RIT_F30:
    def __init__(self, tire_pressure_psi=8.0):
        # Mass & Geometry Parameters
        self.mass = 270.0                           # [sheet] Total mass with driver (kg): 202 kg car + 68 kg driver
        self.unsprung_mass_corner = 7.76643991      # [ASSUMED = SR_17] Wheel/tire/upright/brake mass, per corner (kg)
        self.sprung_mass = self.mass - 4 * self.unsprung_mass_corner
        self.rotational_inertia_factor_front = 1.02 # [ASSUMED = SR_17] Spin-up correction, undriven front wheels

        self.wheelbase = 1.578                      # [sheet] Wheelbase L (m)
        self.cg_height = 0.312941529                # [ASSUMED = SR_17] CG height (m) - not in the sheet
        self.static_rear_bias = 0.50                # [sheet] Front mass distribution 50% -> 50% on the rear axle
        self.wheel_radius = 0.2032                  # [sheet] Tyre radius 203.2 mm

        # Aerodynamics Parameters (the sheet's OpenLAP signs: negative CL = downforce, negative CD = drag)
        self.air_density = 1.225                    # [sheet] kg/m^3
        self.frontal_area = 1.0                     # [sheet] m^2 (OpenLAP's reference area; CL and CD below are per this)
        self.cd = 1.19                              # [sheet] Drag coefficient (sheet -1.19, scale multiplier 1)
        self.cl = 3.408                             # [sheet] Lift coefficient, downforce positive (sheet -3.408, multiplier 1)
        self.aero_rear_bias = 0.53                  # [sheet] Front aero distribution 47% -> 53% on the rear

        # Tires (see the module docstring: the sheet's OpenLAP tyre model isn't used)
        self.tire_pressure_psi = tire_pressure_psi  # Constructor argument, as SR_17 (tire model and kt derive from it)
        self.tires_camber_sensitive = False
        self.crr = 0.015                            # [ASSUMED = SR_17/SR_18] Rolling resistance (the sheet's -0.001 is
                                                    # OpenLAP's recommended default, not a measurement)

        # Suspension Parameters - Front           [ASSUMED = SR_17, all]
        self.spring_rate_front = 70050.73
        self.motion_ratio_front = 0.85
        self.damping_ratio_front = 0.76
        self.anti_lift_front = 0.11
        self.anti_dive_front = 0.33

        # Suspension Parameters - Rear            [ASSUMED = SR_17, all]
        self.spring_rate_rear = 80208.09
        self.motion_ratio_rear = 0.87
        self.damping_ratio_rear = 0.68
        self.anti_squat_rear = 0.44
        self.anti_lift_rear = 0.11

        # Suspension Parameters - shared
        # Tire vertical spring rate: the Hoosier R20 16x7.5-10 TTC measurement SR_17 uses [ASSUMED = SR_17;
        # no LC0 16" spring-rate table is in the data we have]
        self.tire_vertical_spring_rate = np.interp(self.tire_pressure_psi, [8.0, 10.0, 12.0, 14.0],
                                                   [76914.5, 91344.7, 97458.4, 105783.4])  # N/m
        self.track_width = 1.2065                   # [ASSUMED = SR_17] Track width (m)
        self.roll_stiffness_front = 0.45            # [ASSUMED = SR_17] Front share of lateral load transfer

        # Drivetrain & Motor Parameters (single-speed RWD electric)
        self.final_drive_ratio = 3.63               # [sheet] Final gear reduction 3.63 (primary reduction 1, one gear of 1)
        self.drivetrain_efficiency = 0.90           # [sheet] Primary 1.0 x final 1.0 x gearbox 0.9
        self.throttle_request = 1.0
        self.speed_limit_rpm = 5433.5               # [sheet] Torque curve ends at 5433 rpm; no torque past it
        self.power_limit = 80000.0                  # [ASSUMED = SR_17] FSAE 80 kW at the accumulator. The sheet's torque
                                                    # curve already tops out at ~71 kW at the motor shaft.
        self.battery_to_motor_efficiency = 0.97     # [ASSUMED = SR_17] Pack -> motor (wiring + inverter)
        self.rotor_inertia = 0.018                  # [ASSUMED = SR_17] Motor rotor inertia (kg*m^2)
        self.half_shaft_stiffness = 3000.0          # [ASSUMED = SR_17] Rear half-shaft torsional stiffness (Nm/rad)
        self.half_shaft_damping = 15.0              # [ASSUMED = SR_17] (Nm*s/rad)
        self.wheel_inertia_rear = 0.45              # [ASSUMED = SR_17] Rear wheel+tire+hub inertia, both sides (kg*m^2)

        # Slip Control (traction / launch control)  [ASSUMED = SR_17, all]
        self.enable_launch_control = True
        self.target_slip_ratio = 0.10
        self.slip_control_gain = 12.0

        # Motor Torque Curve [sheet, "Torque Curve"]: 230 Nm flat to ~2,900 rpm, then falling at
        # roughly constant power (~71 kW at the shaft) to 125.6 Nm at 5,433 rpm, the last point.
        # The sheet's own sample points, with zero torque just past the last one.
        self.rpm_points = np.array([
            0.669972683, 50.66205277, 157.9218215, 265.1815902, 372.441359, 479.7011277, 586.9608964,
            694.2206651, 801.7851491, 908.7402026, 1015.999971, 1123.25974, 1230.519509, 1331.075542,
            1438.335311, 1545.595079, 1652.854848, 1760.114617, 1867.374386, 1974.634154, 2081.893923,
            2189.153692, 2296.413461, 2403.368514, 2510.932998, 2612.60632, 2720.759921, 2826.008569,
            2933.268337, 3040.528106, 3140.087902, 3210.749726, 3309.794817, 3415.937297, 3523.197065,
            3630.456834, 3737.716603, 3844.976372, 3945.532405, 4053.706319, 4160.051942, 4267.311711,
            4374.57148, 4480.917103, 4589.091017, 4694.339665, 4801.477548, 4907.518455, 5014.778224,
            5122.037993, 5227.957015, 5326.501927, 5433.456981, 5433.5])
        self.torque_points = np.array([
            230.2535546, 230.2197308, 230.2197308, 230.2197308, 230.2197308, 230.2197308, 230.2197308,
            230.2197308, 230.3330272, 230.2197308, 230.2197308, 230.2197308, 230.2197308, 230.2197308,
            230.2197308, 230.2197308, 230.2197308, 230.2197308, 230.2197308, 230.2197308, 230.2197308,
            230.2197308, 230.2197308, 230.3330272, 230.2197308, 230.158187, 230.0812574,
            229.942784, 228.2642088, 220.2776953, 213.3376885, 208.6552417, 202.4236054, 196.1865995,
            190.2705113, 184.641364, 179.3424494, 174.3909126, 169.9386644, 165.4507095, 161.230621,
            157.2236064, 153.4607535, 149.9403181, 146.6016031, 143.5183743, 140.5572875, 137.7911831,
            135.0915126, 132.5076581, 130.047608, 127.9358716, 125.6093481, 0.0])

        self._tire_model = LC0_V2_Tire_Model(wheel_radius=self.wheel_radius, psi=self.tire_pressure_psi)  # default tire: LC0

    def get_motor_torque(self, rpm):
        """Linearly interpolates motor torque from the RPM curve (0 past the speed limit)."""
        return np.interp(rpm, self.rpm_points, self.torque_points, right=0.0)

    def tire_slip_ratio(self, wheel_omega, vx):
        return self._tire_model.tire_slip_ratio(wheel_omega, vx)

    def tire_forces(self, fz, kappa, alpha=0.0):
        return self._tire_model.tire_forces(fz, kappa, alpha)

    def tire_peak_lateral(self, fz):
        return self._tire_model.peak_lateral(fz)

    def tire_peak_longitudinal(self, fz):
        return self._tire_model.peak_longitudinal(fz)
