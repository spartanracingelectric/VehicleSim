class InverterConfig: #TODO: 
    def __init__(self, max_current_a, con_current_a, max_dc_voltage_v):                      
        self.max_current_a = max_current_a                
        self.con_current_a = con_current_a                
        self.max_dc_voltage_v = max_dc_voltage_v       
        self.current_a = 0.0

    def current_update(self, requested_torque_Nm, motor_kt):

        # // Current calculation
        requested_current_a = requested_torque_Nm / motor_kt

        # // limit 
        self.current_a = max(
            -self.max_current_a,
            min(self.max_current_a, requested_current_a)
        )

        return self.current_a

    def has_overcurrent_fault(self):
        if abs(self.current_a) > self.con_current_a:
            raise ValueError(
                f"Inverter overcurrent fault: {self.current_a} A exceeds {self.con_current_a} A"
            )

        return False

