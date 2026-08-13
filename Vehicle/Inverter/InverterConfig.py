class InverterConfig: #TODO: # // don't let longGPT cook this shit alone PLS 
    def __init__(self, mass_kg, max_current_a, con_current_a, max_dc_voltage_v):
        self.max_current_a = max_current_a                # // peak current
        self.con_current_a = con_current_a                # // continuous current
        self.max_dc_voltage_v = max_dc_voltage_v          # // max dc input

    def inverterCalc(self, i, sim):

        # // retrieve info from sim 
        torque_command_Nm = sim.torque_command_Nm[i]
        motor_speed_rad_s = sim.motor_speed_rad_s[i]
        motor_kt = sim.motor.kt # // Chat said something about Id/Iq control table

        # // current command calculation
        requested_motor_current_a = torque_command_Nm / motor_kt
        motor_current_a = max(
            -self.max_current_a,
            min(self.max_current_a, requested_motor_current_a)
        )

        achievable_torque_Nm = motor_current_a * motor_kt

        mechanical_power_W = achievable_torque_Nm * motor_speed_rad_s

        # // todo: efficiency if we're doing that

        # // update sim
        sim.achievable_torque_Nm[i] = achievable_torque_Nm
        sim.inverter_current_a[i] = current_a
        sim.requested_power_W[i] = mechanical_power_W

    # // clamp

    def has_overcurrent_fault(self, i, sim):
        return abs(sim.inverter_current_a[i]) > self.con_current_a

    def has_overvoltage_fault(self, i, sim):
        return sim.terminal_voltage_V[i] > self.max_dc_voltage_v

    # TODO: "Bhuv if you're reading this then hop on arc raiders, also I need BMS current limit"