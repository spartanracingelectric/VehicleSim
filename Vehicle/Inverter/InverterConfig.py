import math
import numpy as np
from Functions.clarkePark import inverseClarke, inversePark
from Functions.PIDcontroller import PIDController

# The inverter runs VESC firmware (SR-Custom-Inverter-FW), so this follows its FOC current loop:
# mcpwm_foc_set_current -> the limit block in mcpwm_foc.c -> control_current -> foc_svm.
# the defaults are what the firmware compiles in (hwconf/other/hw_SRINVERTER.h, then motor/mcconf_default.h),
# a config overrides what gets set in VESC Tool. currents are A peak, VESC's are amplitude-invariant too

# largest voltage vector svm can make without overmodulation: V_bus / sqrt(3) ~ 0.577 V_bus
MAX_VOLTAGE_RATIO = 1 / np.sqrt(3)

# which speed terms the current controller feeds forward (foc_cc_decoupling)
DECOUPLING_MODES = ("none", "cross", "bemf", "cross_bemf")

class InverterConfig:
    #   max_abs_current_A:   l_abs_current_max, the firmware faults above this (fault not modeled yet)
    #   max_dc_voltage_V:    l_max_vin
    #   current_max_A / current_min_A: l_current_max / l_current_min, motor current (min is braking, negative)
    #   in_current_max_A / in_current_min_A: l_in_current_max / l_in_current_min, battery current per inverter
    #   current_kp (V/A), current_ki (V/As): foc_current_kp / foc_current_ki, one pi for both axes
    #   f_zv_Hz, sample_v0_v7: foc_f_zv / foc_sample_v0_v7. center-aligned pwm switches at f_zv / 2,
    #                        and sampling in both zero vectors runs the current loop twice per period
    #   max_duty, vd_max_fraction: l_max_duty / foc_mag_vd_max, margin under the voltage limit
    #   decoupling: foc_cc_decoupling, see DECOUPLING_MODES
    #   max_erpm, erpm_start: l_max_erpm / l_erpm_start, driving current fades to 0 from erpm_start of max_erpm
    #   fw_current_max_A, fw_duty_start, fw_q_current_factor: foc_fw_current_max / foc_fw_duty_start /
    #                        foc_fw_q_current_factor, field weakening (0 A = off)
    def __init__(self, max_abs_current_A=180.0, max_dc_voltage_V=590.0, efficiency=0.97,
                 current_max_A=60.0, current_min_A=-60.0, in_current_max_A=60.0, in_current_min_A=-60.0,
                 current_kp=0.03, current_ki=50.0, f_zv_Hz=26000.0, sample_v0_v7=True,
                 max_duty=0.95, vd_max_fraction=0.98, decoupling="none", max_erpm=100000.0, erpm_start=0.8,
                 fw_current_max_A=0.0, fw_duty_start=0.8, fw_q_current_factor=0.05):
        if not 0 < efficiency <= 1:
            raise ValueError("Efficiency must be between 0 and 1")
        if f_zv_Hz <= 0:
            raise ValueError("f_zv must be positive")
        if not 0 < max_duty <= 1 or not 0 < vd_max_fraction <= 1:
            raise ValueError("max_duty and vd_max_fraction must be between 0 and 1")
        if decoupling not in DECOUPLING_MODES:
            raise ValueError(f"Decoupling must be one of {DECOUPLING_MODES}")
        if not 0 <= erpm_start < 1:
            raise ValueError("erpm_start must be between 0 and 1")
        if current_max_A <= 0 or current_min_A > 0:
            raise ValueError("current_max_A must be positive and current_min_A negative (braking)")

        self.max_abs_current_A = max_abs_current_A
        self.max_dc_voltage_V = max_dc_voltage_V
        self.efficiency = efficiency
        self.switching_frequency_Hz = f_zv_Hz / 2
        self.switching_period_s = 1 / self.switching_frequency_Hz
        self.control_frequency_Hz = f_zv_Hz if sample_v0_v7 else f_zv_Hz / 2
        self.control_period_s = 1 / self.control_frequency_Hz

        self.current_max_A = current_max_A
        self.current_min_A = current_min_A
        self.in_current_max_A = in_current_max_A
        self.in_current_min_A = in_current_min_A
        self.max_duty = max_duty
        self.vd_max_fraction = vd_max_fraction
        self.decoupling = decoupling
        self.max_erpm = max_erpm
        self.erpm_start = erpm_start
        self.fw_current_max_A = fw_current_max_A
        self.fw_duty_start = fw_duty_start
        self.fw_q_current_factor = fw_q_current_factor

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

        # current controller, one pi per axis with the same gains
        self.d_current_controller = PIDController(Kp=current_kp, Ki=current_ki)
        self.q_current_controller = PIDController(Kp=current_kp, Ki=current_ki)
        self.torque_command_Nm = 0.0    # what the current target makes, for the record (VESC has no torque setpoint)
        self.id_desired_A = 0.0
        self.iq_desired_A = 0.0

        # filtered versions of the last period's output, like the firmware keeps:
        # duty for field weakening (m_duty_abs_filtered, 0.01 per loop),
        # mod_q for the battery current limit and the driving direction (mod_q_filter, 0.2 per loop)
        self.duty_filtered = 0.0
        self.mod_q_filtered = 0.0

        # current sensing: averaged over each control period (oversampled, like a real inverter's current
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

    # // Voltage limit: biggest voltage vector the inverter can make from this bus voltage,
    # less the max_duty margin
    def getMaxVoltage_V(self, bus_voltage_V):
        return MAX_VOLTAGE_RATIO * self.max_duty * bus_voltage_V

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

        # zero-sequence voltage that sits the phases inside the bus:
        # centring the min and max phase gives the same average voltages as svm
        v_abc_V = np.array(inverseClarke(v_alpha_V, v_beta_V))
        v_0_V = bus_voltage_V / 2 - (v_abc_V.max() + v_abc_V.min()) / 2

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

    # average current since the last control period (or the motor's current right now if nothing was measured)
    def takeMeasuredCurrents_A(self, motor):
        if self.measured_samples == 0:
            return motor.id, motor.iq
        id_A = self.measured_id_sum_A / self.measured_samples
        iq_A = self.measured_iq_sum_A / self.measured_samples
        self.measured_id_sum_A = 0.0
        self.measured_iq_sum_A = 0.0
        self.measured_samples = 0
        return id_A, iq_A

    # // Current control: current request -> d/q voltage command -> updateVoltage
    # told a current like a VESC's set-current (A peak): that's the iq target, the torque is whatever it makes.
    # reads the motor's currents, speed and angle, like the real inverter reads its current sensors and encoder
    # uses the motor's Ld, Lq and lambda_f for the decoupling, like the firmware's motor config
    # call once per control period
    def updateCurrentControl(self, current_request_A, motor, bus_voltage_V, control_period_s):
        max_voltage_V = self.getMaxVoltage_V(bus_voltage_V)

        # field weakening and the current limits, in the firmware's order
        self.id_desired_A = self.getFieldWeakeningCurrent_A()
        self.id_desired_A, self.iq_desired_A = self.limitCurrents(motor, self.id_desired_A, current_request_A)

        # the torque that current makes, for the record
        self.torque_command_Nm = self.iq_desired_A / motor.find_iq_desired(1.0, self.id_desired_A)

        id_A, iq_A = self.takeMeasuredCurrents_A(motor)
        error_d_A = self.id_desired_A - id_A
        error_q_A = self.iq_desired_A - iq_A

        # feedforward cancels the speed terms in the motor's voltage equations (whichever decoupling says)
        # v_d has -we*Lq*iq so it needs - to cancel
        we = motor.find_we(motor.wm)
        feedforward_d_V = 0.0
        feedforward_q_V = 0.0
        if self.decoupling in ("cross", "cross_bemf"):
            feedforward_d_V -= we * motor.Lq * iq_A
            feedforward_q_V += we * motor.Ld * id_A
        if self.decoupling in ("bemf", "cross_bemf"):
            feedforward_q_V += we * motor.lambda_f

        vd_command_V = self.d_current_controller.getOutput(error_d_A, control_period_s, feedforward_d_V)
        vq_command_V = self.q_current_controller.getOutput(error_q_A, control_period_s, feedforward_q_V)

        # the d axis gets its voltage first (it's what field weakening works through, capped at vd_max_fraction
        # of the limit) and the q axis gets whatever's left, instead of shrinking both and losing control of id
        vd_max_V = self.vd_max_fraction * max_voltage_V
        vq_max_V = math.sqrt(max_voltage_V ** 2 - min(abs(vd_command_V), vd_max_V) ** 2)
        saturated = abs(vd_command_V) > vd_max_V or abs(vq_command_V) > vq_max_V
        vd_command_V = max(-vd_max_V, min(vd_command_V, vd_max_V))
        vq_command_V = max(-vq_max_V, min(vq_command_V, vq_max_V))

        # the voltage is held for a whole period while the rotor keeps turning, so aim it at where the rotor
        # will be halfway through (the firmware moves its observer angle ahead by speed * dt * 0.5)
        theta_e_rad = motor.find_theta_e(motor.theta_m)
        theta_command_rad = theta_e_rad + we * control_period_s / 2

        # makes the phase voltages (already inside the limit). saturated is whether the command had to be cut
        self.updateVoltage(vd_command_V, vq_command_V, theta_command_rad, bus_voltage_V)
        self.voltage_saturated = saturated

        # anti-windup: only integrate when the inverter could make the voltage asked for
        self.d_current_controller.updateState(error_d_A, control_period_s, integrate=not self.voltage_saturated)
        self.q_current_controller.updateState(error_q_A, control_period_s, integrate=not self.voltage_saturated)

        # what the output actually was, filtered, for next period.
        # duty: 1 = the biggest vector the bus can make (duty_now), mod_q: vq / (2/3 V_bus)
        duty = math.hypot(vd_command_V, vq_command_V) / (MAX_VOLTAGE_RATIO * bus_voltage_V)
        mod_q = vq_command_V * 1.5 / bus_voltage_V
        self.duty_filtered += 0.01 * (duty - self.duty_filtered)
        self.mod_q_filtered += 0.2 * (mod_q - self.mod_q_filtered)

        return self.phase_voltages_V

    # field weakening (foc_run_fw): 0 below fw_duty_start of max_duty, then a straight line on the filtered duty
    # up to fw_current_max_A at max_duty. negative id
    def getFieldWeakeningCurrent_A(self):
        duty_start = self.fw_duty_start * self.max_duty
        if self.fw_current_max_A <= 0 or self.duty_filtered <= duty_start:
            return 0.0
        fraction = (self.duty_filtered - duty_start) / (self.max_duty - duty_start)
        return -self.fw_current_max_A * min(fraction, 1.0)

    # current limits, in the firmware's order (mcpwm_foc.c, after field weakening):
    # field weakening takes a bit off iq, then the battery current limit, then the motor current limits
    # (the driving one faded near max speed), then the total current under the biggest limit.
    # mod_q's sign says which way the inverter is driving: driving current is iq with the same sign
    def limitCurrents(self, motor, id_A, iq_A):
        mod_q = self.mod_q_filtered
        iq_A -= math.copysign(1.0, mod_q) * -id_A * self.fw_q_current_factor

        # battery current ~ mod_q * iq, so the battery limit is a limit on iq
        if mod_q > 0.001:
            iq_A = max(self.in_current_min_A / mod_q, min(iq_A, self.in_current_max_A / mod_q))
        elif mod_q < -0.001:
            iq_A = max(self.in_current_max_A / mod_q, min(iq_A, self.in_current_min_A / mod_q))

        # the speed limit fades only the driving limit, braking isn't faded
        driving_max_A = self.current_max_A * self.getSpeedFade(motor)
        if mod_q > 0:
            iq_A = max(self.current_min_A, min(iq_A, driving_max_A))
        else:
            iq_A = max(-driving_max_A, min(iq_A, -self.current_min_A))

        # keep the total current under the biggest limit, id first
        current_max_abs_A = max(self.current_max_A, -self.current_min_A)
        id_A = max(-current_max_abs_A, min(id_A, current_max_abs_A))
        iq_max_A = math.sqrt(max(current_max_abs_A ** 2 - id_A ** 2, 0.0))
        iq_A = max(-iq_max_A, min(iq_A, iq_max_A))
        return id_A, iq_A

    # speed limit (l_max_erpm, l_erpm_start): 1 normally, a straight line down to 0 from erpm_start of max_erpm
    # erpm = electrical rpm = mechanical rpm * pole pairs
    def getSpeedFade(self, motor):
        erpm = abs(motor.wm) * 60 / (2 * math.pi) * motor.Np / 2
        fade = (self.max_erpm - erpm) / ((1 - self.erpm_start) * self.max_erpm)
        return max(0.0, min(fade, 1.0))

    # contactors open, so there's no dc bus: the inverter can't drive anything.
    # clears its outputs and the controllers' memory so it starts fresh if the bus comes back
    def turnOff(self):
        self.d_current_controller.reset()
        self.q_current_controller.reset()
        self.duty_filtered = 0.0
        self.mod_q_filtered = 0.0
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

    #TODO: the max_abs_current_A fault (mc_interface: FAULT_CODE_ABS_OVER_CURRENT)

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
