import numpy as np
class InverterConfig:
    def __init__(self, con_current_A, max_dc_voltage_V):                                   
        self.con_current_A = con_current_A                
        self.max_dc_voltage_V = max_dc_voltage_V
        self.current_A = 0.0
        
    def update(self, requested_torque_Nm, motor_kt, battery_max_current_A: float):
        self.updateCurrent(requested_torque_Nm, motor_kt, battery_max_current_A)

#TODO: 1. send a request to the battery to limit the current instead of getting battery current limit
    def updateCurrent(self, requested_torque_Nm, motor_kt, battery_max_current_A: float):

        # // Current calculation
        requested_current_A = requested_torque_Nm / motor_kt * 1000
        
        # // limit 
        self.current_A = np.clip(requested_current_A, -battery_max_current_A, battery_max_current_A)

        return self.current_A

#TODO: 2. con_current pid ramp up current :D
    # def has_overcurrent_fault(self):
    #     return abs(self.current_A) > self.con_current_A
    
    def getCurrent_A(self):
        return self.current_A

