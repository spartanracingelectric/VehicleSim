from Functions.constants import G_MPS2, AIR_DENSITY_KGPM3

class VehicleConfig:
    # num_motors: how many motor + inverter pairs share the battery (each motor has its own inverter)
    # gear_ratio: motor turns per wheel turn (wheel torque = motor torque * gear_ratio)
    # drag_area_m2: drag coefficient * frontal area (Cd * A), rolling_resistance_coeff: rolling force / weight
    def __init__(self, vehicle_mass_kg, axel, battery, bms, brake, driver, inverter, motor, tire,
                 vcu=None, num_motors=1, gear_ratio=1.0, drag_area_m2=0.0, rolling_resistance_coeff=0.0):
        self.vehicle_mass_kg = vehicle_mass_kg
        self.axel = axel
        self.battery = battery
        self.bms = bms
        self.brake = brake
        self.driver = driver
        self.inverter = inverter
        self.motor = motor
        self.tire = tire
        self.vcu = vcu
        self.num_motors = num_motors
        
        # temporary here because we're missing other models
        self.gear_ratio = gear_ratio
        self.drag_area_m2 = drag_area_m2
        self.rolling_resistance_coeff = rolling_resistance_coeff

        # per motor, averaged over the motor's small steps in one update. everything else lives in its own part:
        # vcu.torque_request_Nm, inverter.torque_command_Nm, battery.current_A, battery.terminal_power_W
        # torque request / command / feedback line up with the log's MCM Torque Command / Commanded Torque / Torque Feedback
        self.torque_feedback_Nm = 0.0    # what the motor actually made (its Te)
        self.average_copper_loss_W = 0.0

        # chassis state, starts parked
        self.speed_mps = 0.0
        self.distance_m = 0.0
        self.traction_limited = False

    # One update = one inverter switching period:
    #   pedal -> vcu (runs its own 10 ms loop, holds the request in between)
    #   -> inverter (caps the torque at the motor's max, then current control)
    #   -> motor (small electrical steps) -> inverter dc current -> battery -> bms reads the pack
    #   -> chassis: motor torque through the gearbox moves the car, which sets the motor speed for next time
    #
    # the 80 kW rule isn't enforced here, the flashed VCU has its own power limiting algorithm.
    # the bms just flags overpower so you can see when a request would break the rule
    #
    # every motor gets the same torque request, so one motor + inverter pair is simulated
    # and its dc current is multiplied by num_motors for the battery
    # TODO: simulate each pair on its own once motors get different requests (torque vectoring)
    #
    # dt_s is the motor's electrical timestep inside the period
    def update(self, apps_percent, dt_s=5e-6):
        control_period_s = self.getControlPeriod_s()
        torque_request_Nm = self.vcu.update(apps_percent, control_period_s)
        motor_speed_radps = self.getMotorSpeed_radps()

        # a bms shutdown from an earlier period keeps the contactors open
        if self.bms.isShutdown():
            self.updateContactorsOpen(motor_speed_radps)
        else:
            self.updateDriving(torque_request_Nm, motor_speed_radps, dt_s)

        # bms reads the pack once per its own sample period (bms.sample_period_s, 50 ms), not every tick:
        # voltage/temperature faults latch a shutdown, overpower is only flagged
        self.bms.updateFromPack(self.battery, control_period_s)

        self.updateChassis(control_period_s)

        return self.torque_feedback_Nm

    def updateDriving(self, torque_request_Nm, motor_speed_radps, dt_s):
        control_period_s = self.getControlPeriod_s()
        steps_per_control_period = round(control_period_s / dt_s)
        if steps_per_control_period < 1:
            raise ValueError("Motor timestep must be shorter than the inverter switching period")

        bus_voltage_V = self.battery.terminal_voltage_V
        self.motor.set_wm(motor_speed_radps)
        self.inverter.updateCurrentControl(torque_request_Nm, self.motor, bus_voltage_V, control_period_s)

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

    # straight line: motor torque through the gearbox pushes the car, drag and rolling resistance hold it back.
    # every wheel is driven, so the tires can push with up to mu * weight. past that the extra torque just spins
    # the wheels, that's flagged but not modeled (the real car's traction control is in the VCU firmware)
    # TODO: wheel slip, load transfer, gearbox losses, rotating inertia of the motors and wheels
    def updateChassis(self, dt_s):
        mass_kg = self.getMass_kg()
        weight_N = mass_kg * G_MPS2

        wheel_force_N = self.num_motors * self.torque_feedback_Nm * self.gear_ratio / self.tire.radius_m
        grip_N = self.tire.tiremu * weight_N
        self.traction_limited = abs(wheel_force_N) > grip_N
        drive_force_N = max(-grip_N, min(wheel_force_N, grip_N))

        drag_N = 0.5 * AIR_DENSITY_KGPM3 * self.drag_area_m2 * self.speed_mps ** 2
        rolling_N = self.rolling_resistance_coeff * weight_N if self.speed_mps > 0 else 0.0

        acceleration_mps2 = (drive_force_N - drag_N - rolling_N) / mass_kg
        self.speed_mps = max(0.0, self.speed_mps + acceleration_mps2 * dt_s)
        self.distance_m += self.speed_mps * dt_s

    # same as simrunner: car + battery
    # TODO: driver isn't in here (driver1.mass_kg is still a placeholder)
    def getMass_kg(self):
        return self.vehicle_mass_kg + self.battery.mass_kg

    # wheels turn with the car (no slip), motors turn gear_ratio times faster
    def getMotorSpeed_radps(self):
        return self.speed_mps / self.tire.radius_m * self.gear_ratio

    # one update covers one inverter switching period
    def getControlPeriod_s(self):
        return self.inverter.switching_period_s

    # what the motor actually made: its Te, per motor, averaged over the update
    def getTorqueFeedback_Nm(self):
        return self.torque_feedback_Nm

    # per motor, averaged over the update
    def getAverageCopperLoss_W(self):
        return self.average_copper_loss_W

    def getSpeed_mps(self):
        return self.speed_mps

    def getDistance_m(self):
        return self.distance_m

    def isTractionLimited(self):
        return self.traction_limited
