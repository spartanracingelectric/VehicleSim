from Functions.constants import G_MPS2, AIR_DENSITY_KGPM3

class ChassisConfig:
    # straight line: the wheels push the car, drag and rolling resistance hold it back.
    # drag_area_m2: drag coefficient * frontal area (Cd * A), rolling_resistance_coeff: rolling force / weight
    def __init__(self, drag_area_m2, rolling_resistance_coeff):
        self.drag_area_m2 = drag_area_m2
        self.rolling_resistance_coeff = rolling_resistance_coeff

        # starts parked
        self.speed_mps = 0.0
        self.distance_m = 0.0
        self.traction_limited = False

    # wheel_force_N: what the motors push with at the road, all wheels together
    # every wheel is driven, so the tires can push with up to mu * weight. past that the extra torque just spins
    # the wheels, that's flagged but not modeled (the real car's traction control is in the VCU firmware)
    # TODO: wheel slip, load transfer, rotating inertia of the motors and wheels
    def update(self, wheel_force_N, mass_kg, tire, dt_s):
        weight_N = mass_kg * G_MPS2

        grip_N = tire.tiremu * weight_N
        self.traction_limited = abs(wheel_force_N) > grip_N
        drive_force_N = max(-grip_N, min(wheel_force_N, grip_N))

        acceleration_mps2 = (drive_force_N - self.getDragForce_N() - self.getRollingResistance_N(weight_N)) / mass_kg
        self.speed_mps = max(0.0, self.speed_mps + acceleration_mps2 * dt_s)
        self.distance_m += self.speed_mps * dt_s

    def getDragForce_N(self):
        return 0.5 * AIR_DENSITY_KGPM3 * self.drag_area_m2 * self.speed_mps ** 2

    # only while rolling, so a parked car doesn't get pushed backwards
    def getRollingResistance_N(self, weight_N):
        return self.rolling_resistance_coeff * weight_N if self.speed_mps > 0 else 0.0

    def getSpeed_mps(self):
        return self.speed_mps

    def getDistance_m(self):
        return self.distance_m

    def isTractionLimited(self):
        return self.traction_limited
