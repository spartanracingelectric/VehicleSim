class VehicleConfig:
    # num_motors: how many motor + inverter pairs share the battery (each motor has its own inverter)
    # gear_ratio: motor turns per wheel turn (wheel torque = motor torque * gear_ratio)
    def __init__(self, vehicle_mass_kg, axel, battery, bms, brake, chassis, driver, inverter, motor, tire,
                 vcu=None, num_motors=1, gear_ratio=1.0):
        self.vehicle_mass_kg = vehicle_mass_kg
        self.axel = axel
        self.battery = battery
        self.bms = bms
        self.brake = brake
        self.chassis = chassis
        self.driver = driver
        self.inverter = inverter
        self.motor = motor
        self.tire = tire
        self.vcu = vcu
        self.num_motors = num_motors

        # temporary here because we're missing other models
        self.gear_ratio = gear_ratio

        # per motor, averaged over the motor's small steps in one update. everything else lives in its own part:
        # vcu.current_request_A, inverter.torque_command_Nm, battery.current_A, battery.terminal_power_W,
        # chassis.speed_mps
        # torque command / feedback line up with the log's MCM Commanded Torque / Torque Feedback
        self.torque_feedback_Nm = 0.0    # what the motor actually made (its Te)
        self.average_copper_loss_W = 0.0

    # One update = one run of the inverter's current loop (twice per switching period, sampling in v0 and v7):
    #   pedal -> vcu (runs its own 10 ms loop, holds the current request in between)
    #   -> inverter (VESC: current limits, field weakening, then current control)
    #   -> motor (small electrical steps) -> inverter dc current -> battery -> bms reads the pack
    #   -> chassis: motor torque through the gearbox moves the car, which sets the motor speed for next time
    #
    # the 80 kW rule isn't enforced here, the flashed VCU has its own power limiting algorithm.
    # the bms just flags overpower so you can see when a request would break the rule
    #
    # every motor gets the same current request, so one motor + inverter pair is simulated
    # and its dc current is multiplied by num_motors for the battery
    # TODO: simulate each pair on its own once motors get different requests (torque vectoring)
    #
    # dt_s is the motor's electrical timestep inside the period.
    # current_request_A (per motor, A peak, the VESC's set-current) skips the car's own vcu, for when something
    # else decides, e.g. the real VCU firmware in a SIL test (vcu.getCurrentRequest_A() then isn't updated)
    def update(self, apps_percent=0.0, dt_s=5e-6, current_request_A=None):
        control_period_s = self.getControlPeriod_s()
        if current_request_A is None:
            current_request_A = self.vcu.update(apps_percent, control_period_s)
        motor_speed_radps = self.getMotorSpeed_radps()

        # a bms shutdown from an earlier period keeps the contactors open
        if self.bms.isShutdown():
            self.updateContactorsOpen(motor_speed_radps)
        else:
            self.updateDriving(current_request_A, motor_speed_radps, dt_s)

        # bms reads the pack once per its own sample period (bms.sample_period_s, 50 ms), not every tick:
        # voltage/temperature faults latch a shutdown, overpower is only flagged
        self.bms.updateFromPack(self.battery, control_period_s)

        self.chassis.update(self.getWheelForce_N(), self.getMass_kg(), self.tire, control_period_s)

        return self.torque_feedback_Nm

    # runs the car for duration_s: as many updates (inverter control periods, 38.5 us at 26 kHz) as fit, e.g. 260 for
    # one 10 ms VCU loop. same arguments as update(), held for the whole time
    def advance(self, duration_s, apps_percent=0.0, current_request_A=None):
        # round, not int: duration / period often lands a hair under a whole number in floating point
        for _ in range(round(duration_s / self.getControlPeriod_s())):
            self.update(apps_percent, current_request_A=current_request_A)
        return self.torque_feedback_Nm

    def updateDriving(self, current_request_A, motor_speed_radps, dt_s):
        control_period_s = self.getControlPeriod_s()
        steps_per_control_period = round(control_period_s / dt_s)
        if steps_per_control_period < 1:
            raise ValueError("Motor timestep must be shorter than the inverter control period")
        # stretched a little so the steps fill the period exactly (5 us doesn't divide a 26 kHz loop's 38.5 us)
        dt_s = control_period_s / steps_per_control_period

        bus_voltage_V = self.battery.terminal_voltage_V
        self.motor.set_wm(motor_speed_radps)
        self.inverter.updateCurrentControl(current_request_A, self.motor, bus_voltage_V, control_period_s)

        # motor runs in small steps while the inverter holds its voltages for the period
        torque_sum_Nm = 0.0
        copper_loss_sum_W = 0.0
        dc_current_sum_A = 0.0
        for _ in range(steps_per_control_period):
            # the car sets the motor's speed (through the wheels and gearbox)
            _, _, Te, _, _, Pe = self.motor.update_from_phase_voltages(self.inverter.getPhaseVoltages_V(), dt_s, wm=motor_speed_radps)
            self.inverter.measureCurrents(self.motor.id, self.motor.iq)
            self.inverter.updatePower(Pe, bus_voltage_V)

            torque_sum_Nm += Te
            copper_loss_sum_W += self.motor.P_copper
            dc_current_sum_A += self.inverter.getDcCurrent_A()

        self.torque_feedback_Nm = torque_sum_Nm / steps_per_control_period
        self.average_copper_loss_W = copper_loss_sum_W / steps_per_control_period

        # the battery sees every inverter's dc current, averaged over the period
        pack_current_A = self.num_motors * dc_current_sum_A / steps_per_control_period
        self.battery.update(pack_current_A, control_period_s)

    # contactors open: no dc bus, so the inverters can't drive any current and the motors just spin with the car
    def updateContactorsOpen(self, motor_speed_radps):
        control_period_s = self.getControlPeriod_s()
        self.motor.turnOff(control_period_s, wm=motor_speed_radps)
        self.inverter.turnOff()

        self.torque_feedback_Nm = 0.0
        self.average_copper_loss_W = 0.0
        self.battery.update(0.0, control_period_s)

    # motor torque through the gearbox and tires, all motors together. the chassis caps it at the tires' grip
    # TODO: gearbox losses
    def getWheelForce_N(self):
        return self.num_motors * self.torque_feedback_Nm * self.gear_ratio / self.tire.radius_m

    # same as simrunner: car + battery
    # TODO: driver isn't in here (driver1.mass_kg is still a placeholder)
    def getMass_kg(self):
        return self.vehicle_mass_kg + self.battery.mass_kg

    # wheels turn with the car (no slip), motors turn gear_ratio times faster
    def getMotorSpeed_radps(self):
        return self.chassis.getSpeed_mps() / self.tire.radius_m * self.gear_ratio

    # one update covers one run of the inverter's current loop (twice per switching period, sampling in v0 and v7)
    def getControlPeriod_s(self):
        return self.inverter.control_period_s

    # what the motor actually made: its Te, per motor, averaged over the update
    def getTorqueFeedback_Nm(self):
        return self.torque_feedback_Nm

    # per motor, averaged over the update
    def getAverageCopperLoss_W(self):
        return self.average_copper_loss_W
