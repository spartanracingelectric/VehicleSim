class InverterConfig: #TODO: # // Don't let longGPT cook this shit alone PLS 
    def __init__(self, mass_kg, max_current_a, con_current_a, max_dc_voltage_v):
        #// self.mass_kg = mass_kg                        # // saw this in battery code, idk what it's used for
        self.max_current_a = max_current_a                # // peak current
        self.con_current_a = con_current_a                # // continuous current
        self.max_dc_voltage_v = max_dc_voltage_v          # // max dc input
        # // self.efficiency = efficiency                 # // idk if we're doing efficiency  

    def inverterCalc(self, i, sim):

        # // retrieve info from sim 
        torque_command_Nm = sim.torque_command_Nm[i]
        motor_speed_rad_s = sim.motor_speed_rad_s[i]
        motor_kt = sim.motor.kt #// constant torque, also Mario what is kt?

        # // torque -> current, limited to inverter's peak current limit
        requested_current_a = torque_command_Nm / motor_kt
        current_a = max(-self.max_current_a, min(self.max_current_a, requested_current_a))

        achievable_torque_Nm = current_a * motor_kt
        mechanical_power_W = achievable_torque_Nm * motor_speed_rad_s
        # // no efficiency yet - power in = power out for now

        # // update sim
        sim.achievable_torque_Nm[i] = achievable_torque_Nm
        sim.inverter_current_a[i] = current_a
        sim.requested_power_W[i] = mechanical_power_W

    # // clamp

    def has_overcurrent_fault(self, i, sim):
        return abs(sim.inverter_current_a[i]) > self.con_current_a

    def has_overvoltage_fault(self, i, sim):
        return sim.terminal_voltage_V[i] > self.max_dc_voltage_v