import math
import numbers


class VehicleConfig:
    # num_motors: how many motor + inverter pairs share the battery (each motor has its own inverter)
    # gear_ratio: motor turns per wheel turn (wheel torque = motor torque * gear_ratio)
    def __init__(self, vehicle_mass_kg, axel, battery, bms, brake, chassis, driver, inverters, motors, tire,
                 vcu=None, num_motors=4, gear_ratio=1.0):
        self.vehicle_mass_kg = vehicle_mass_kg
        self.axel = axel
        self.battery = battery
        self.bms = bms
        self.brake = brake
        self.chassis = chassis
        self.driver = driver
        self.inverters = list(inverters)
        self.motors = list(motors) # gets 4 motors: front left, front right, rear left, rear right
        self.tire = tire
        self.vcu = vcu
        self.num_motors = num_motors
        if len(self.motors) != num_motors or len(self.inverters) != num_motors:
            raise ValueError(f"Need one motor and one inverter per motor: num_motors={num_motors}, "
                             f"got {len(self.motors)} motors and {len(self.inverters)} inverters")
        if len({id(m) for m in self.motors}) != num_motors or len({id(i) for i in self.inverters}) != num_motors:
            raise ValueError("Each corner needs independent motor and inverter instances")
        if len({i.control_period_s for i in self.inverters}) != 1:
            raise ValueError("All inverters must use the same control period")

        # temporary here because we're missing other models
        self.gear_ratio = gear_ratio

        # per motor, averaged over the motor's small steps in one update. everything else lives in its own part:
        # vcu.torque_request_Nm, inverter.torque_command_Nm, battery.current_A, battery.terminal_power_W,
        # chassis.speed_mps
        # torque request / command / feedback line up with the log's MCM Torque Command / Commanded Torque / Torque Feedback
        self.torque_feedback_Nm: list[float] = [0.0] * self.num_motors   # what the motor actually made (its Te)
        self.average_copper_loss_W: list[float] = [0.0] * self.num_motors

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
        # Scalars broadcast to all motors; sequences use FL, FR, RL, RR order.
        # Current requests are A peak at the motor, not battery current or RMS.
    #
    # dt_s is the motor's electrical timestep inside the period.
    # torque_request_Nm or current_request_A (per motor) skip the car's own vcu, for when something else decides,
    # e.g. the real VCU firmware in a SIL test (vcu.getTorqueRequest_Nm() then isn't updated):
    #   torque_request_Nm: the inverters are told a torque
    #   current_request_A: the inverters are told a current (A peak, like a VESC's set-current), see InverterConfig
    def update(self, apps_percent=0.0, dt_s=5e-6, torque_request_Nm=None, current_request_A=None, *, bps_percent=0.0,
               steering_angle_rad=0.0):
        control_period_s = self.getControlPeriod_s()
        if torque_request_Nm is not None and current_request_A is not None:
            raise ValueError("Set current_request_A or torque_request_Nm, not both")
        if torque_request_Nm is None and current_request_A is None:
            if self.vcu is None:
                raise ValueError("External current/torque commands are required without a stand-in VCU")
            current_request_A = self.vcu.update(apps_percent, control_period_s)
        currents = self._corner_requests(current_request_A) if current_request_A is not None else None
        torques = self._corner_requests(torque_request_Nm) if torque_request_Nm is not None else None
        motor_speed_radps = self.getMotorSpeed_radps()

        # a bms shutdown from an earlier period keeps the contactors open
        if self.bms.isShutdown():
            self.updateContactorsOpen(motor_speed_radps)
        else:
            self.updateDriving(torques, motor_speed_radps, dt_s, currents)

        # bms reads the pack once per its own sample period (bms.sample_period_s, 50 ms), not every tick:
        # voltage/temperature faults latch a shutdown, overpower is only flagged
        self.bms.updateFromPack(self.battery, control_period_s)

        # negative motor torque is regen braking: the friction brakes only make up the rest of the pedal
        self.brake.update(bps_percent, regen_torque_Nm=max(0.0, -self.getWheelTorque_Nm()))
        self.chassis.update(self.getWheelTorque_Nm(), self.brake.getFrontWheelTorque_Nm(), self.brake.getRearWheelTorque_Nm(),
                            self.getDrivelineInertia_kgm2(), self.getMass_kg(), self.tire, control_period_s,
                            steering_angle_rad=steering_angle_rad)

        return self.torque_feedback_Nm

    # runs the car for duration_s: as many updates (inverter switching periods, 100 us) as fit, e.g. 100 for
    # one 10 ms VCU loop. same arguments as update(), held for the whole time
    def advance(self, duration_s, apps_percent=0.0, torque_request_Nm=None, current_request_A=None, *, bps_percent=0.0,
                steering_angle_rad=0.0):
        # round, not int: 0.01 / 0.0001 is 99.99999999999999 in floating point
        for _ in range(round(duration_s / self.getControlPeriod_s())):
            self.update(apps_percent, torque_request_Nm=torque_request_Nm, current_request_A=current_request_A,
                        bps_percent=bps_percent, steering_angle_rad=steering_angle_rad)
        return self.torque_feedback_Nm

    def updateDriving(self, torque_request_Nm, motor_speed_radps, dt_s, current_request_A=None):
        control_period_s = self.getControlPeriod_s()
        steps_per_control_period = round(control_period_s / dt_s)
        if steps_per_control_period < 1:
            raise ValueError("Motor timestep must be shorter than the inverter switching period")
        # The motor substeps must fill the 26 kHz VESC control period exactly.
        dt_s = control_period_s / steps_per_control_period

        bus_voltage_V = self.battery.terminal_voltage_V
        pack_current_A = 0.0

        for i in range(self.num_motors):
            self.motors[i].set_wm(motor_speed_radps)
            request_A = (current_request_A[i] if current_request_A is not None else
                         self.motors[i].find_iq_desired(torque_request_Nm[i]))
            self.inverters[i].updateCurrentControl(request_A, self.motors[i], bus_voltage_V, control_period_s)

            # motor runs in small steps while the inverter holds its voltages for the period
        
            torque_sum_Nm = 0.0
            copper_loss_sum_W = 0.0
            dc_current_sum_A = 0.0

            for _ in range(steps_per_control_period):
                # the car sets the motor's speed (through the wheels and gearbox)
                _, _, Te, _, _, Pe = self.motors[i].update_from_phase_voltages(self.inverters[i].getPhaseVoltages_V(), dt_s, wm=motor_speed_radps)
                self.inverters[i].measureCurrents(self.motors[i].id, self.motors[i].iq)
                self.inverters[i].updatePower(Pe, bus_voltage_V)

                torque_sum_Nm += Te
                copper_loss_sum_W += self.motors[i].P_copper
                dc_current_sum_A += self.inverters[i].getDcCurrent_A()

            self.torque_feedback_Nm[i] = torque_sum_Nm / steps_per_control_period
            self.average_copper_loss_W[i] = copper_loss_sum_W / steps_per_control_period

            # the battery sees every inverter's dc current, averaged over the period
            pack_current_A += dc_current_sum_A / steps_per_control_period
        self.battery.update(pack_current_A, control_period_s)

    # contactors open: no dc bus, so the inverters can't drive any current and the motors just spin with the car
    def updateContactorsOpen(self, motor_speed_radps):
        control_period_s = self.getControlPeriod_s()
        for i in range(self.num_motors):
            self.motors[i].turnOff(control_period_s, wm=motor_speed_radps)
            self.inverters[i].turnOff()

            self.torque_feedback_Nm[i] = 0.0
            self.average_copper_loss_W[i] = 0.0
        self.battery.update(0.0, control_period_s)

    # motor torque through the gearbox, per wheel. every wheel is driven, the motors are shared out evenly
    # (4 hub motors: one each). the tires turn it into force in the chassis
    # the chassis still gives every wheel the same torque, so the motors' torques are averaged
    # TODO: each wheel gets its own motor's torque once the chassis has a wheel per corner (torque vectoring)
    # TODO: gearbox losses
    def getWheelTorque_Nm(self):
        return sum(self.torque_feedback_Nm) * self.gear_ratio / self.getWheelCount()

    # the motors' rotors spin gear_ratio times faster than the wheel, so each one adds Jm * gear_ratio^2 at the wheel
    def getDrivelineInertia_kgm2(self):
        return sum(motor.Jm for motor in self.motors) * self.gear_ratio ** 2 / self.getWheelCount()

    def getWheelCount(self):
        return 2 * self.chassis.wheels_per_axle

    # same as simrunner: car + battery
    # TODO: driver isn't in here (driver1.mass_kg is still a placeholder)
    def getMass_kg(self):
        return self.vehicle_mass_kg + self.battery.mass_kg

    # motors turn gear_ratio times faster than the wheels (which can slip now, so not the car's speed)
    # every motor spins at the front and rear wheels' average for now
    # TODO: each motor at its own wheel's speed
    def getMotorSpeed_radps(self):
        return self.chassis.getWheelSpeed_radps(self.tire.radius_m) * self.gear_ratio

    # one update covers one inverter switching period
    # assuming control period is same for all inverters, so it uses first inverter
    def getControlPeriod_s(self):
        return self.inverters[0].control_period_s

    def _corner_requests(self, value):
        requests = [float(value)] * self.num_motors if isinstance(value, numbers.Real) else list(value)
        if len(requests) != self.num_motors or not all(math.isfinite(v) for v in requests):
            raise ValueError(f"Expected {self.num_motors} finite per-motor commands in FL, FR, RL, RR order")
        return requests

    # what each motor actually made: its Te, averaged over the update. a list, one per motor
    def getTorqueFeedback_Nm(self):
        return self.torque_feedback_Nm

    # per motor, averaged over the update
    def getAverageCopperLoss_W(self):
        return self.average_copper_loss_W
