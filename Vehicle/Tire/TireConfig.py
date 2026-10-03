# This configuration file defines the parameters for a specific tire model used in the
# vehicle simulation. The TireConfig class encapsulates the properties and behaviors of a tire,
# including its radius and stiffness characteristics. Based on Dugoff tire model, not Magic Formula
#
# NOTE: there is a second dugoff model, Vehicle/Tire/TireConfig2.py on VehSim-1 (nelsonng1).
# This one is meant to replace it, still needs to be agreed on with nelson before merging.
# Tire values in Configs/tire1.py come from his TTC data (Configs/tiretesting2.py on VehSim-1).
# Differences: his slip ratio divides by wheel speed and uses 1/(1 - k), which mixes two slip
# conventions (his kappa/(1 - kappa) is our k, so his linear Fx is Cx k instead of Cx k / (1 + k))
import numpy

# Radius must be > 0, stiffnesses and friction can't be negative (None means not set yet)
# Stiffness is either fixed (N per unit slip, N/rad) or per load (multiplied by Fz), not both
class TireConfig: #TODO: Add parameters related to tire

    tireSlipRatio = 0.0 # longitudinal slip ratio
    tireSlipAngle = 0.0 # lateral slip angle
    tireDownForce = 0.0 # downward force on the tire
    tireLongitudinalForce = 0.0 # longitudinal force on the tire
    tireLateralForce = 0.0 # lateral force on the tire
    tireCouplingFactor = 0.0 # dugoff lambda, dimensionless, how close the tire is to the friction limit

    def __init__(self, radius_m, longitudinal_stiffness=None, lateral_stiffness=None, tireroadfriction=None,
                 longitudinal_stiffness_per_load=None, lateral_stiffness_per_load=None):
        if radius_m <= 0:
            raise ValueError(f"radius_m must be > 0, got {radius_m}")
        if longitudinal_stiffness is not None and longitudinal_stiffness < 0:
            raise ValueError(f"longitudinal_stiffness can't be negative, got {longitudinal_stiffness}")
        if lateral_stiffness is not None and lateral_stiffness < 0:
            raise ValueError(f"lateral_stiffness can't be negative, got {lateral_stiffness}")
        if tireroadfriction is not None and tireroadfriction < 0:
            raise ValueError(f"tireroadfriction can't be negative, got {tireroadfriction}")
        if longitudinal_stiffness_per_load is not None and longitudinal_stiffness_per_load < 0:
            raise ValueError(f"longitudinal_stiffness_per_load can't be negative, got {longitudinal_stiffness_per_load}")
        if lateral_stiffness_per_load is not None and lateral_stiffness_per_load < 0:
            raise ValueError(f"lateral_stiffness_per_load can't be negative, got {lateral_stiffness_per_load}")
        if longitudinal_stiffness is not None and longitudinal_stiffness_per_load is not None:
            raise ValueError("set longitudinal_stiffness or longitudinal_stiffness_per_load, not both")
        if lateral_stiffness is not None and lateral_stiffness_per_load is not None:
            raise ValueError("set lateral_stiffness or lateral_stiffness_per_load, not both")

        self.radius_m = radius_m # radius of the tire in meters
        self.longitudinal_stiffness = longitudinal_stiffness # forward backwards motion
        self.lateral_stiffness = lateral_stiffness # sideways motion pushing on the tire
        self.tireroadfriction = tireroadfriction # friction between tire and road, can be set later
        self.longitudinal_stiffness_per_load = longitudinal_stiffness_per_load # longitudinal stiffness / Fz, stiffness grows with load
        self.lateral_stiffness_per_load = lateral_stiffness_per_load # lateral stiffness / Fz, stiffness grows with load

    def calculateTireSlipAngle(self, wheel_angle, wheel_longitudinal_velocity, wheel_lateral_velocity):
        # Is the wheel pointing in the direction it should be pointing compared to the actual vehicle movement?
        self.tireSlipAngle = wheel_angle - numpy.arctan(wheel_lateral_velocity / wheel_longitudinal_velocity) if wheel_longitudinal_velocity != 0 else 0.0
        return self.tireSlipAngle

    def calculateTireSlipRatio(self, wheel_speed, vehicle_speed):
        # Is the wheel rotating at the speed it should be rotating compared to the actual vehicle movement?
        # driving > 0, braking < 0, locked wheel = -1
        self.tireSlipRatio = (wheel_speed - vehicle_speed) / vehicle_speed if vehicle_speed != 0 else 0.0
        return self.tireSlipRatio

    def calculateTireLongitudinalForce(self):
        # Cx (k) / (1 + k) * f(lambda) where k is the slip ratio/longitudinal slip
        if self.tireSlipRatio == -1:
            self.tireLongitudinalForce = self.getLockedWheelForce(self.getLinearLongitudinalForce())
            return self.tireLongitudinalForce
        saturating_factor = self.getSaturation()
        self.tireLongitudinalForce = self.getLinearLongitudinalForce() / (1 + self.tireSlipRatio) * saturating_factor
        return self.tireLongitudinalForce

    def calculateTireLateralForce(self):
        # Cy tan(alpha) / (1 + k) * f(lambda) where alpha is the slip angle
        if self.tireSlipRatio == -1:
            self.tireLateralForce = self.getLockedWheelForce(self.getLinearLateralForce())
            return self.tireLateralForce
        saturating_factor = self.getSaturation()
        self.tireLateralForce = self.getLinearLateralForce() / (1 + self.tireSlipRatio) * saturating_factor
        return self.tireLateralForce

    def calculateCoupling(self, tireDownForce):
        # lambda = mu Fz (1 + k) / (2 sqrt((Cx k)^2 + (Cy tan(alpha))^2))
        # lambda >= 1 means the tire is still in the linear region, < 1 means it's sliding
        self.tireDownForce = tireDownForce
        linear_force_magnitude = self.getLinearForceMagnitude()
        if not self.tireroadfriction or tireDownForce == 0:
            self.tireCouplingFactor = 0.0 # no grip, no force
        elif linear_force_magnitude == 0:
            self.tireCouplingFactor = numpy.inf # no slip, nowhere near the limit
        else:
            self.tireCouplingFactor = (self.tireroadfriction * tireDownForce * (1 + self.tireSlipRatio)) / (2 * linear_force_magnitude)
        return self.tireCouplingFactor

    def getSaturation(self):
        # f(lambda) = lambda (2 - lambda) if lambda < 1, else 1
        if (self.tireCouplingFactor < 1.0):
            return ((2 * self.tireCouplingFactor) - (self.tireCouplingFactor**2))
        return 1

    def getLockedWheelForce(self, linear_force):
        # Locked wheel (k = -1) is 0/0 in the formula, the limit is full sliding friction mu Fz
        # split between x and y in the same direction as the linear forces
        linear_force_magnitude = self.getLinearForceMagnitude()
        if not self.tireroadfriction or linear_force_magnitude == 0:
            return 0.0
        return self.tireroadfriction * self.tireDownForce * linear_force / linear_force_magnitude

    def getLongitudinalStiffness(self):
        # Cx at the current load, fixed stiffness or per load stiffness * Fz
        if self.longitudinal_stiffness_per_load is not None:
            return self.longitudinal_stiffness_per_load * self.tireDownForce
        return self.longitudinal_stiffness

    def getLateralStiffness(self):
        # Cy at the current load, fixed stiffness or per load stiffness * Fz
        if self.lateral_stiffness_per_load is not None:
            return self.lateral_stiffness_per_load * self.tireDownForce
        return self.lateral_stiffness

    def getLinearLongitudinalForce(self):
        # Cx k, force the tire would make with infinite grip
        longitudinal_stiffness = self.getLongitudinalStiffness()
        return longitudinal_stiffness * self.tireSlipRatio if longitudinal_stiffness else 0.0

    def getLinearLateralForce(self):
        # Cy tan(alpha), force the tire would make with infinite grip
        lateral_stiffness = self.getLateralStiffness()
        return lateral_stiffness * numpy.tan(self.tireSlipAngle) if lateral_stiffness else 0.0

    def getLinearForceMagnitude(self):
        return numpy.hypot(self.getLinearLongitudinalForce(), self.getLinearLateralForce())

    def getTireSlipRatio(self):
        return self.tireSlipRatio
    def getTireSlipAngle(self):
        return self.tireSlipAngle
