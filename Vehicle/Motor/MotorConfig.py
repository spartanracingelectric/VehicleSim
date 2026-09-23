import math
import numpy as np

class MotorConfig: #TODO: Add parameters related to motor
    # init function runs automatically when you create a MotorConfig object
    def __init__(self, Kt, peakCurrent_A, peakTorque_Nm, peak_power_W, max_rpm):
        self.Kt = Kt #torque constant
        self.peakCurrent_A = peakCurrent_A
        self.peakTorque_Nm = peakTorque_Nm
        self.peak_power_W = peak_power_W
        self.max_rpm = max_rpm

    # update the motor’s state using the current and speed
    def update(self, inverter_current_A, rpm):
        self.updateTorque(inverter_current_A, rpm)

    # This calculates the motor torque for the current timestep
    def updateTorque(self, inverter_current_A, rpm):
        current_A = np.clip(inverter_current_A, -self.peakCurrent_A, self.peakCurrent_A)
        torque_Nm = self.Kt * current_A
        torque_Nm = np.clip(torque_Nm, -self.peakTorque_Nm, self.peakTorque_Nm)
        velocity_rad_s = abs(rpm) * 2.0 * math.pi / 60.0
        
        if velocity_rad_s > 0:
            max_torque_Nm = self.peak_power_W / velocity_rad_s
            torque_limit = min(self.peakTorque_Nm, max_torque_Nm)
            torque_Nm = np.clip(torque_Nm, -torque_limit, torque_limit)
            
        return torque_Nm

    # returns motor constant, Kt
    def getMotor_kt(self):
        return self.Kt