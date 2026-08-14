import math

class MotorConfig: #TODO: Add parameters related to motor
    def __init__(self, Kt, peakCurrent_A, peakTorque_Nm, peak_power_W, max_rpm):
        self.Kt = Kt #torque constant
        self.peakCurrent_A = peakCurrent_A
        self.peakTorque_Nm = peakTorque_Nm
        self.peak_power_W = 124000
        self.max_rpm = 6500
        
    def updateTorque(self, inverter_current_A, rpm):
        current_A = max(-self.peakCurrent_A, min(inverter_current_A, self.peakCurrent_A))
        torque_Nm = self.Kt * current_A
        torque_Nm = max(-self.peakTorque_Nm, min(torque_Nm, self.peakTorque_Nm))
        velocity_rad_s = abs(rpm) * 2.0 * math.pi / 60.0
        
        if velocity_rad_s > 0:
            max_torque_Nm = self.peak_power_W / velocity_rad_s
            torque_limit = min(self.peakTorque, max_torque_Nm)
            torque_Nm = max(-torque_limit, min(torque_Nm, torque_limit))
            
        return torque_Nm