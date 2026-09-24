import math
import numpy as np

class MotorConfig: #TODO: Add parameters related to motor
    # init function runs automatically when you create a MotorConfig object
    def __init__(self, Kt, Rs, Np, Lq, Ld, Jm):
        self.Kt = Kt #torque constant
        # self.peakCurrent_A = peakCurrent_A
        # self.peakTorque_Nm = peakTorque_Nm
        # self.peak_power_W = peak_power_W
        # self.max_rpm = max_rpm
        self.Rs = Rs #stator winding resistance
        self.Np = Np #number of poles
        self.Lq = Lq #quadrature axis inductance (mH)
        self.Ld = Ld #direct axis inductance (mH)
        self.Jm = Jm #inertia (kgcm^2??)

    # update the motor’s state using the current and speed
    # recieve vd and vq from inverter
    def update(self, vd, vq):
        #self.updateTorque(inverter_current_A, rpm)
        pass

    def find_we(self, old_wm):
         # (Np/2) number of pole pairs
        return (self.Np/2)*old_wm
    
    def update_id(self, vd, old_id, we, old_iq):
        # find did/dt then update id
        
        pass

    
    def update_iq(self, vq, old_iq, we, old_id, lambda_f):
         # find diq/dt then update iq
        pass


    def find_Te(self, lambda_f, updated_iq, updated_id):
        pass


    def update_wm(self, Te, T_load, Bm, old_wm, T_friction):
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