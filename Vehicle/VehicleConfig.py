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
        # vcu.torque_request_Nm, inverter.torque_command_Nm, battery.current_A, battery.terminal_power_W,
        # chassis.speed_mps
        # torque request / command / feedback line up with the log's MCM Torque Command / Commanded Torque / Torque Feedback
        self.torque_feedback_Nm = 0.0    # what the motor actually made (its Te)
        self.average_copper_loss_W = 0.0

    # One update = one inverter switching period:
    #   pedal -> vcu (runs its own 10 ms loop, holds the request in between)
    #   -> inverter (caps the torque at the motor's max, then current control)
    #   -> motor (small electrical steps) -> inverter dc current -> battery -> bms reads the pack
    #   -> chassis: motor torque through the gearbox spins the wheels, the brakes (brake pedal, BPS %) hold them back,
    #      the tires push the car. the wheels' speed sets the motor speed for next time
    #
    # the 80 kW rule isn't enforced here, the flashed VCU has its own power limiting algorithm.
    # the bms just flags overpower so you can see when a request would break the rule
    #
    # every motor gets the same torque request, so one motor + inverter pair is simulated
    # and its dc current is multiplied by num_motors for the battery
    # TODO: simulate each pair on its own once motors get different requests (torque vectoring)
    #
    # dt_s is the motor's electrical timestep inside the period.
    # torque_request_Nm or current_request_A (per motor) skip the car's own vcu, for when something else decides,
    # e.g. the real VCU firmware in a SIL test (vcu.getTorqueRequest_Nm() then isn't updated):
    #   torque_request_Nm: the inverters are told a torque
    #   current_request_A: the inverters are told a current (A peak, like a VESC's set-current), see InverterConfig
    def update(self, apps_percent=0.0, dt_s=5e-6, torque_request_Nm=None, current_request_A=None, *, bps_percent=0.0):
        control_period_s = self.getControlPeriod_s()
        if torque_request_Nm is None and current_request_A is None:
            torque_request_Nm = self.vcu.update(apps_percent, control_period_s)
        motor_speed_radps = self.getMotorSpeed_radps()

        # a bms shutdown from an earlier period keeps the contactors open
        if self.bms.isShutdown():
            self.updateContactorsOpen(motor_speed_radps)
        else:
            self.updateDriving(torque_request_Nm, motor_speed_radps, dt_s, current_request_A)

        # bms reads the pack once per its own sample period (bms.sample_period_s, 50 ms), not every tick:
        # voltage/temperature faults latch a shutdown, overpower is only flagged
        self.bms.updateFromPack(self.battery, control_period_s)

        self.brake.update(bps_percent)
        self.chassis.update(self.getWheelTorque_Nm(), self.brake.getFrontWheelTorque_Nm(), self.brake.getRearWheelTorque_Nm(),
                            self.getDrivelineInertia_kgm2(), self.getMass_kg(), self.tire, control_period_s)

        return self.torque_feedback_Nm

    # runs the car for duration_s: as many updates (inverter switching periods, 100 us) as fit, e.g. 100 for
    # one 10 ms VCU loop. same arguments as update(), held for the whole time
    def advance(self, duration_s, apps_percent=0.0, torque_request_Nm=None, current_request_A=None, *, bps_percent=0.0):
        # round, not int: 0.01 / 0.0001 is 99.99999999999999 in floating point
        for _ in range(round(duration_s / self.getControlPeriod_s())):
            self.update(apps_percent, torque_request_Nm=torque_request_Nm, current_request_A=current_request_A,
                        bps_percent=bps_percent)
        return self.torque_feedback_Nm

    def updateDriving(self, torque_request_Nm, motor_speed_radps, dt_s, current_request_A=None):
        control_period_s = self.getControlPeriod_s()
        steps_per_control_period = round(control_period_s / dt_s)
        if steps_per_control_period < 1:
            raise ValueError("Motor timestep must be shorter than the inverter switching period")

        bus_voltage_V = self.battery.terminal_voltage_V
        self.motor.set_wm(motor_speed_radps)
        self.inverter.updateCurrentControl(torque_request_Nm, self.motor, bus_voltage_V, control_period_s,
                                           current_request_A)

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

    # motor torque through the gearbox, per wheel. every wheel is driven, the motors are shared out evenly
    # (4 hub motors: one each). the tires turn it into force in the chassis
    # TODO: gearbox losses
    def getWheelTorque_Nm(self):
        return self.num_motors * self.torque_feedback_Nm * self.gear_ratio / self.getWheelCount()

    # the motors' rotors spin gear_ratio times faster than the wheel, so each one adds Jm * gear_ratio^2 at the wheel
    def getDrivelineInertia_kgm2(self):
        return self.num_motors * self.motor.Jm * self.gear_ratio ** 2 / self.getWheelCount()

    def getWheelCount(self):
        return 2 * self.chassis.wheels_per_axle

    # same as simrunner: car + battery
    # TODO: driver isn't in here (driver1.mass_kg is still a placeholder)
    def getMass_kg(self):
        return self.vehicle_mass_kg + self.battery.mass_kg

    # motors turn gear_ratio times faster than the wheels (which can slip now, so not the car's speed)
    # one motor is simulated for all of them, so it spins at the front and rear wheels' average
    # TODO: front and rear motors at their own wheel's speed once each pair is simulated on its own
    def getMotorSpeed_radps(self):
        return self.chassis.getWheelSpeed_radps(self.tire.radius_m) * self.gear_ratio

    # one update covers one inverter switching period
    def getControlPeriod_s(self):
        return self.inverter.switching_period_s

    # what the motor actually made: its Te, per motor, averaged over the update
    def getTorqueFeedback_Nm(self):
        return self.torque_feedback_Nm

    # per motor, averaged over the update
    def getAverageCopperLoss_W(self):
        return self.average_copper_loss_W
