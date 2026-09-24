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
    # recieve vd and vq from inverter
    def update(self, vd, vq):
        #self.updateTorque(inverter_current_A, rpm)
        pass

    def find_we(self, Np, old_wm):
         # (Np/2) number of pole pairs
        return (Np/2)*old_wm
    
    def update_id(self, vd, Rs, old_id, we, Lq, old_iq, Ld):
        # find did/dt then update id
        pass

    
    def update_iq(self, vq, Rs, old_iq, we, Ld, old_id, lambda_f, Lq):
         # find diq/dt then update iq
        pass


    def find_Te(self, Np, lambda_f, updated_iq, Ld, Lq, updated_id):
        pass


    def update_wm(self, Te, T_load, Bm, old_wm, T_friction, Jm):
        pass





    # # This calculates the motor torque for the current timestep
    # def updateTorque(self, inverter_current_A, rpm):
    #     current_A = np.clip(inverter_current_A, -self.peakCurrent_A, self.peakCurrent_A)
    #     torque_Nm = self.Kt * current_A
    #     torque_Nm = np.clip(torque_Nm, -self.peakTorque_Nm, self.peakTorque_Nm)
    #     velocity_rad_s = abs(rpm) * 2.0 * math.pi / 60.0
        
    #     if velocity_rad_s > 0:
    #         max_torque_Nm = self.peak_power_W / velocity_rad_s
    #         torque_limit = min(self.peakTorque_Nm, max_torque_Nm)
    #         torque_Nm = np.clip(torque_Nm, -torque_limit, torque_limit)
            
    #     return torque_Nm

    # returns motor constant, Kt
    def getMotor_kt(self):
        return self.Kt