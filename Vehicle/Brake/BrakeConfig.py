import math

class BrakeConfig:
    # hydraulic friction brakes, pedal (BPS %) -> line pressure -> caliper clamp -> pad friction -> torque at each wheel
    #
    #   pedal force   = bps / 100 * max_pedal_force_N
    #   push rods     = pedal force * pedal_ratio, split front / rear by the balance bar (front_bias)
    #   line pressure = push rod force / master cylinder bore area (one master cylinder per axle)
    #   wheel torque  = 2 pad faces * pad_mu * line pressure * piston area on one side * rotor_radius_m
    #
    # the torque is how hard the brake can grip the rotor, it only ever slows the wheel down to a stop and never
    # spins it backwards, the chassis does that part (Vehicle/Chassis/ChassisConfig.py)
    # TODO: hydraulic lag, pad mu changing with temperature, regen (that's the VCU firmware)
    def __init__(self, mass_kg, max_pedal_force_N, pedal_ratio, front_bias,
                 front_master_cylinder_bore_m, rear_master_cylinder_bore_m,
                 front_caliper_piston_diameter_m, front_caliper_pistons_per_side,
                 rear_caliper_piston_diameter_m, rear_caliper_pistons_per_side,
                 pad_mu, front_rotor_radius_m, rear_rotor_radius_m):
        if not 0 <= front_bias <= 1:
            raise ValueError(f"front_bias must be between 0 and 1, got {front_bias}")
        for name, value in (("max_pedal_force_N", max_pedal_force_N), ("pedal_ratio", pedal_ratio),
                            ("front_master_cylinder_bore_m", front_master_cylinder_bore_m),
                            ("rear_master_cylinder_bore_m", rear_master_cylinder_bore_m),
                            ("front_caliper_piston_diameter_m", front_caliper_piston_diameter_m),
                            ("rear_caliper_piston_diameter_m", rear_caliper_piston_diameter_m),
                            ("front_rotor_radius_m", front_rotor_radius_m), ("rear_rotor_radius_m", rear_rotor_radius_m)):
            if value <= 0:
                raise ValueError(f"{name} must be > 0, got {value}")
        if pad_mu < 0:
            raise ValueError(f"pad_mu can't be negative, got {pad_mu}")

        self.mass_kg = mass_kg
        self.max_pedal_force_N = max_pedal_force_N    # what the driver pushes with at 100% pedal
        self.pedal_ratio = pedal_ratio                # push rod force / pedal force
        self.front_bias = front_bias                  # balance bar: fraction of the push rod force going to the front
        self.front_master_cylinder_area_m2 = circleArea_m2(front_master_cylinder_bore_m)
        self.rear_master_cylinder_area_m2 = circleArea_m2(rear_master_cylinder_bore_m)
        self.front_piston_area_m2 = front_caliper_pistons_per_side * circleArea_m2(front_caliper_piston_diameter_m)
        self.rear_piston_area_m2 = rear_caliper_pistons_per_side * circleArea_m2(rear_caliper_piston_diameter_m)
        self.pad_mu = pad_mu                          # pad on rotor friction
        self.front_rotor_radius_m = front_rotor_radius_m    # effective radius, middle of the pad
        self.rear_rotor_radius_m = rear_rotor_radius_m

        # brakes off
        self.bps_percent = 0.0
        self.regen_torque_Nm = 0.0
        self.front_pressure_Pa = 0.0
        self.rear_pressure_Pa = 0.0

    # call every sim step with the brake pedal
    # regen_torque_Nm: braking torque the motors are already putting on each wheel (>= 0, per wheel).
    # The pedal is the total the driver asked for, so the friction brakes only make up the rest (blended braking).
    # The line pressure is still the full pedal pressure (that's what the pressure sensor reads)
    def update(self, bps_percent, regen_torque_Nm=0.0):
        self.bps_percent = max(0.0, min(bps_percent, 100.0))
        self.regen_torque_Nm = max(0.0, regen_torque_Nm)
        push_rod_force_N = self.bps_percent / 100 * self.max_pedal_force_N * self.pedal_ratio
        self.front_pressure_Pa = push_rod_force_N * self.front_bias / self.front_master_cylinder_area_m2
        self.rear_pressure_Pa = push_rod_force_N * (1 - self.front_bias) / self.rear_master_cylinder_area_m2

    # total braking the pedal is asking for, per wheel (regen + friction), what a regen controller blends against
    def getFrontRequestedTorque_Nm(self):
        return 2 * self.pad_mu * self.front_pressure_Pa * self.front_piston_area_m2 * self.front_rotor_radius_m

    def getRearRequestedTorque_Nm(self):
        return 2 * self.pad_mu * self.rear_pressure_Pa * self.rear_piston_area_m2 * self.rear_rotor_radius_m

    # what the friction brakes still have to hold, per wheel: the request minus what regen already provides
    def getFrontWheelTorque_Nm(self):
        return max(0.0, self.getFrontRequestedTorque_Nm() - self.regen_torque_Nm)

    def getRearWheelTorque_Nm(self):
        return max(0.0, self.getRearRequestedTorque_Nm() - self.regen_torque_Nm)

    # same as the log's Brake Pres Front / Brake Pres Rear (but in Pa)
    def getFrontPressure_Pa(self):
        return self.front_pressure_Pa

    def getRearPressure_Pa(self):
        return self.rear_pressure_Pa

def circleArea_m2(diameter_m):
    return math.pi * diameter_m ** 2 / 4
