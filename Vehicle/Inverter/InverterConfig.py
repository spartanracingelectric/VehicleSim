import math
import numpy as np
from Functions.clarkePark import inverseClarke, inversePark
from Functions.PIDcontroller import PIDController

# largest average voltage vector each modulation can make, as a fraction of bus voltage
MAX_VOLTAGE_RATIO = {
    "spwm": 0.5,              # sinusoidal pwm: V_bus / 2
    "svm": 1 / np.sqrt(3),    # space vector modulation: V_bus / sqrt(3) ~ 0.577 V_bus
}

class InverterConfig:
    def __init__(self, con_current_A, max_dc_voltage_V, efficiency=0.97, modulation="svm", switching_frequency_Hz=10000.0,
                 current_kp_d=0.58, current_kp_q=0.64, current_tn_d_ms=1.2, current_tn_q_ms=1.2,
                 field_weakening_kp_AperV=0.08, field_weakening_tn_ms=6.0, field_weakening_start=0.95):
        if not 0 < efficiency <= 1:
            raise ValueError("Efficiency must be between 0 and 1")
        if modulation not in MAX_VOLTAGE_RATIO:
            raise ValueError(f"Modulation must be one of {list(MAX_VOLTAGE_RATIO)}")
        if switching_frequency_Hz <= 0:
            raise ValueError("Switching frequency must be positive")

        self.con_current_A = con_current_A
        self.max_dc_voltage_V = max_dc_voltage_V
        self.efficiency = efficiency
        self.modulation = modulation
        self.switching_frequency_Hz = switching_frequency_Hz
        self.switching_period_s = 1 / switching_frequency_Hz

        # dc bus side, positive = power flowing out of the battery
        self.bus_voltage_V = 0.0
        self.ac_power_W = 0.0
        self.dc_power_W = 0.0
        self.dc_current_A = 0.0
        self.loss_W = 0.0

        # average-value voltage outputs
        self.voltage_saturated = False
        self.duty_cycles = np.zeros(3)
        self.phase_voltages_V = np.zeros(3)

        # switching outputs, 1 = upper igbt of that leg on
        self.gate_states = np.zeros(3, dtype=int)
        self.switched_voltages_V = np.zeros(3)

        # current controller, one pi per axis. gains are Kp (V/A) and Tn (ms), Ki = Kp / Tn
        self.d_current_controller = PIDController(Kp=current_kp_d, Ki=current_kp_d / (current_tn_d_ms / 1000))
        self.q_current_controller = PIDController(Kp=current_kp_q, Ki=current_kp_q / (current_tn_q_ms / 1000))
        self.torque_command_Nm = 0.0
        self.id_desired_A = 0.0
        self.iq_desired_A = 0.0

        # field weakening ("voltage controller" on the amk datasheet): a pi on how much voltage is left,
        # its output is a negative id that cancels some of the magnet's flux at high speed.
        # starts once the voltage command passes field_weakening_start of what the bus can make.
        # output_max = 0 so it only ever weakens, output_min gets set from the motor's limit
        self.field_weakening_start = field_weakening_start
        self.field_weakening_controller = PIDController(
            Kp=field_weakening_kp_AperV, Ki=field_weakening_kp_AperV / (field_weakening_tn_ms / 1000), output_max=0.0)
        self.voltage_command_V = 0.0    # size of last period's d/q voltage command, before limiting
        self.d_axis_priority = True     # at the voltage limit: d axis first (True) or shrink both evenly (False)

        # current sensing: averaged over each switching period (oversampled, like a real inverter's current
        # sensors), so the controller regulates the average current, not one sample of it
        self.measured_id_sum_A = 0.0
        self.measured_iq_sum_A = 0.0
        self.measured_samples = 0

    # // Level 0: constant efficiency, motor-side ac power -> dc bus power and current
    def updatePower(self, ac_power_W, bus_voltage_V):
        if bus_voltage_V <= 0:
            raise ValueError("Bus voltage must be positive")

        self.bus_voltage_V = bus_voltage_V
        self.ac_power_W = ac_power_W

        # motoring: the battery also pays for the losses
        # regen: the losses come out of what reaches the battery
        if ac_power_W >= 0:
            self.dc_power_W = ac_power_W / self.efficiency
        else:
            self.dc_power_W = ac_power_W * self.efficiency

        self.loss_W = self.dc_power_W - ac_power_W
        self.dc_current_A = self.dc_power_W / bus_voltage_V

        return self.dc_current_A

    # // Voltage limit: biggest voltage vector the inverter can make from this bus voltage
    def getMaxVoltage_V(self, bus_voltage_V):
        return MAX_VOLTAGE_RATIO[self.modulation] * bus_voltage_V

    # scales a d/q voltage command back onto the limit circle, keeping its direction
    # voltage_saturated tells the current controller to stop integrating (anti-windup)
    def limitVoltage(self, v_d_V, v_q_V, bus_voltage_V):
        magnitude_V = np.hypot(v_d_V, v_q_V)
        max_voltage_V = self.getMaxVoltage_V(bus_voltage_V)

        self.voltage_saturated = magnitude_V > max_voltage_V
        if self.voltage_saturated:
            scale = max_voltage_V / magnitude_V
            v_d_V *= scale
            v_q_V *= scale

        return v_d_V, v_q_V

    # // Level 1: average-value voltage source
    # synchronous d/q voltage command -> phase duty cycles -> average phase voltages
    def updateVoltage(self, v_d_syn_V, v_q_syn_V, theta_e_rad, bus_voltage_V):
        if bus_voltage_V <= 0:
            raise ValueError("Bus voltage must be positive")

        self.bus_voltage_V = bus_voltage_V

        # rotor frame -> stationary frame, then keep it inside what the bus can make
        v_alpha_V, v_beta_V = inversePark(v_d_syn_V, v_q_syn_V, theta_e_rad)
        v_alpha_V, v_beta_V = self.limitVoltage(v_alpha_V, v_beta_V, bus_voltage_V)

        # zero-sequence voltage that sits the phases inside the bus
        v_abc_V = np.array(inverseClarke(v_alpha_V, v_beta_V))
        if self.modulation == "svm":
            # centring the min and max phase gives the same average voltages as svm
            v_0_V = bus_voltage_V / 2 - (v_abc_V.max() + v_abc_V.min()) / 2
        else:
            v_0_V = bus_voltage_V / 2

        self.duty_cycles = np.clip((v_abc_V + v_0_V) / bus_voltage_V, 0.0, 1.0)

        # average voltage of each phase leg, measured from the negative rail
        # the zero-sequence part cancels out in a star-connected motor
        self.phase_voltages_V = self.duty_cycles * bus_voltage_V

        return self.phase_voltages_V

    # the inverter's current sensors: call every motor step, updateCurrentControl uses the average over the period
    def measureCurrents(self, id_A, iq_A):
        self.measured_id_sum_A += id_A
        self.measured_iq_sum_A += iq_A
        self.measured_samples += 1

    # average current since the last switching period (or the motor's current right now if nothing was measured)
    def takeMeasuredCurrents_A(self, motor):
        if self.measured_samples == 0:
            return motor.id, motor.iq
        id_A = self.measured_id_sum_A / self.measured_samples
        iq_A = self.measured_iq_sum_A / self.measured_samples
        self.measured_id_sum_A = 0.0
        self.measured_iq_sum_A = 0.0
        self.measured_samples = 0
        return id_A, iq_A

    # // Current control: torque request -> d/q voltage command -> updateVoltage
    # reads the motor's currents, speed and angle, like the real inverter reads its current sensors and resolver
    # uses the motor's Ld, Lq, lambda_f and poles for the feedforward, like a real inverter's motor setup
    # call once per switching period
    def updateCurrentControl(self, torque_request_Nm, motor, bus_voltage_V, control_period_s):
        # won't ask the motor for more torque than it can make
        self.torque_command_Nm = max(-motor.peakTorque_Nm, min(torque_request_Nm, motor.peakTorque_Nm))

        # speed limit: fades driving torque out over the last 2% before the motor's max speed
        speed_rpm = abs(motor.wm) * 60 / (2 * math.pi)
        if motor.maxSpeed_rpm is not None and self.torque_command_Nm * motor.wm > 0:
            fade = (motor.maxSpeed_rpm - speed_rpm) / (0.02 * motor.maxSpeed_rpm)
            self.torque_command_Nm *= max(0.0, min(fade, 1.0))

        # field weakening: if last period's voltage command got near what the bus can make, push id negative.
        # that cancels some of the magnet's flux (less back-emf) so there's voltage left to make torque
        headroom_V = self.field_weakening_start * self.getMaxVoltage_V(bus_voltage_V) - self.voltage_command_V
        self.field_weakening_controller.output_min = -motor.fieldWeakeningCurrent_A
        self.id_desired_A = self.field_weakening_controller.update(headroom_V, control_period_s)

        # torque -> iq, counting the reluctance torque from that id, then keep the total current under the motor's max
        self.iq_desired_A = motor.find_iq_desired(self.torque_command_Nm, self.id_desired_A)
        if motor.maxCurrent_A is not None:
            iq_max_A = math.sqrt(max(motor.maxCurrent_A ** 2 - self.id_desired_A ** 2, 0.0))
            self.iq_desired_A = max(-iq_max_A, min(self.iq_desired_A, iq_max_A))

        id_A, iq_A = self.takeMeasuredCurrents_A(motor)
        error_d_A = self.id_desired_A - id_A
        error_q_A = self.iq_desired_A - iq_A

        # feedforward cancels the speed terms in the motor's voltage equations
        # (pdf has + on the d-axis term, but v_d has -we*Lq*iq so it needs - to cancel)
        we = motor.find_we(motor.wm)
        feedforward_d_V = -we * motor.Lq * iq_A
        feedforward_q_V = we * (motor.Ld * id_A + motor.lambda_f)

        vd_command_V = self.d_current_controller.getOutput(error_d_A, control_period_s, feedforward_d_V)
        vq_command_V = self.q_current_controller.getOutput(error_q_A, control_period_s, feedforward_q_V)
        self.voltage_command_V = math.hypot(vd_command_V, vq_command_V)    # field weakening looks at this next period

        # if the bus can't make that much, the d axis gets its voltage first (it's what field weakening works
        # through) and the q axis gets whatever's left, instead of shrinking both and losing control of id
        max_voltage_V = self.getMaxVoltage_V(bus_voltage_V)
        saturated = self.voltage_command_V > max_voltage_V
        if saturated and self.d_axis_priority:
            vd_command_V = max(-max_voltage_V, min(vd_command_V, max_voltage_V))
            vq_command_V = math.copysign(math.sqrt(max_voltage_V ** 2 - vd_command_V ** 2), vq_command_V)

        # the voltage is held for a whole period while the rotor keeps turning,
        # so aim it at where the rotor will be halfway through the period
        theta_e_rad = motor.find_theta_e(motor.theta_m)
        theta_command_rad = theta_e_rad + we * control_period_s / 2

        # makes the phase voltages (already inside the limit). saturated is whether the command had to be cut
        self.updateVoltage(vd_command_V, vq_command_V, theta_command_rad, bus_voltage_V)
        self.voltage_saturated = saturated

        # anti-windup: only integrate when the inverter could make the voltage asked for
        self.d_current_controller.updateState(error_d_A, control_period_s, integrate=not self.voltage_saturated)
        self.q_current_controller.updateState(error_q_A, control_period_s, integrate=not self.voltage_saturated)

        return self.phase_voltages_V

    # contactors open, so there's no dc bus: the inverter can't drive anything.
    # clears its outputs and the controllers' memory so it starts fresh if the bus comes back
    def turnOff(self):
        self.d_current_controller.reset()
        self.q_current_controller.reset()
        self.field_weakening_controller.reset()
        self.voltage_command_V = 0.0
        self.torque_command_Nm = 0.0
        self.id_desired_A = 0.0
        self.iq_desired_A = 0.0
        self.duty_cycles = np.zeros(3)
        self.phase_voltages_V = np.zeros(3)
        self.ac_power_W = 0.0
        self.dc_power_W = 0.0
        self.dc_current_A = 0.0
        self.loss_W = 0.0

    # // Level 2: pwm modulator
    # triangle carrier, 0 -> 1 -> 0 once per switching period
    def getCarrier(self, time_s):
        carrier_phase = (time_s * self.switching_frequency_Hz) % 1.0
        return 1 - abs(2 * carrier_phase - 1)

    # compares the duty cycles from updateVoltage with the carrier to pick each leg's switch state
    # upper igbt is on while duty > carrier, so it's on for d * T_sw of every period
    # ideal switches with no dead time or losses yet, those are Level 3
    def updateSwitching(self, time_s, phase_currents_A=None):
        carrier = self.getCarrier(time_s)
        self.gate_states = (self.duty_cycles > carrier).astype(int)

        # each leg ties its phase to dc+ (V_bus) or dc- (0 V)
        self.switched_voltages_V = self.gate_states * self.bus_voltage_V

        # the battery only sees current from the phases switched to dc+
        if phase_currents_A is not None:
            self.dc_current_A = float(np.dot(self.gate_states, phase_currents_A))
            self.dc_power_W = self.dc_current_A * self.bus_voltage_V
            self.loss_W = 0.0

        return self.switched_voltages_V

#TODO: 2. con_current pid ramp up current :D
    # def has_overcurrent_fault(self):
    #     return abs(self.current_A) > self.con_current_A

    def getDcCurrent_A(self):
        return self.dc_current_A

    def getDcPower_W(self):
        return self.dc_power_W

    def getLoss_W(self):
        return self.loss_W

    def getDutyCycles(self):
        return self.duty_cycles

    def getPhaseVoltages_V(self):
        return self.phase_voltages_V

    def isVoltageSaturated(self):
        return self.voltage_saturated

    def getTorqueCommand_Nm(self):
        return self.torque_command_Nm

    def getCurrentDesired_A(self):
        return self.id_desired_A, self.iq_desired_A

    # complementary pair for each leg: the lower igbt is always the opposite of the upper
    def getGateCommands(self):
        upper = self.gate_states
        lower = 1 - self.gate_states
        return upper, lower

    def getSwitchedVoltages_V(self):
        return self.switched_voltages_V

