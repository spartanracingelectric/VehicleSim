# This configuration file defines the parameters for a specific tire model used in the 
# vehicle simulation. The TireConfig class encapsulates the properties and behaviors of a tire, 
# including its radius and stiffness characteristics. Based on Dugoff tire model, not Magic Formula
import numpy

# Note, do we check for the right values like zero, or can we input negative? we probably want to check for negative values and throw an error if they are inputted. and also write tests
class TireConfig: #TODO: Add parameters related to tire
    def __init__(self, radius_m, longitudinal_stiffness=None, lateral_stiffness=None):
        self.radius_m = radius_m # radius of the tire in meters
        self.longitudinal_stiffness = longitudinal_stiffness # forward backwards motion
        self.lateral_stiffness = lateral_stiffness # sideways motion pushing on the tire

    def calculateTireSlipAngle(self, wheel_angle, wheel_longitudinal_velocity, wheel_lateral_velocity):
        # Is the wheel pointing in the direction it should be pointing compared to the actual vehicle movement?
        tireSlipAngle = wheel_angle - numpy.arctan(wheel_lateral_velocity / wheel_longitudinal_velocity) if wheel_longitudinal_velocity != 0 else 0.0

        return tireSlipAngle

    def calculateTireSlipRatio(self, wheel_speed, vehicle_speed):
        # Placeholder implementation for slip ratio calculation
        # Is the wheel rotating at the speed it should be rotating compared to the actual vehicle movement?
        if vehicle_speed == 0:
            return 0.0
        return (wheel_speed - vehicle_speed) / vehicle_speed

    def calculateTireLongitudinalForce(self, slip_ratio):
        # Cx (k) / (1 + k) where k is the slip ratio
        tireLongitudinalForce = (self.longitudinal_stiffness * slip_ratio) / (1 + slip_ratio) if self.longitudinal_stiffness and slip_ratio != -1 else 0.0
        return tireLongitudinalForce

    def calculateTireLateralForce(self, slip_angle):
        # Cy (alpha) / (1 + alpha) where alpha is the slip angle
        tireLateralForce = (self.lateral_stiffness * slip_angle) / (1 + slip_angle) if self.lateral_stiffness and slip_angle != -1 else 0.0
        return tireLateralForce

    

    