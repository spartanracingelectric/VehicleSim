import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))    # repo root, so this also runs as a file or with the run button
import math
import numpy as np
from copy import deepcopy
from Vehicle.Inverter.Configs.inverter1 import inverter1
from Vehicle.Motor.Configs.amk_dd5 import amk_dd5

# run: python Tests/motorThermalTest.py (or python -m Tests.motorThermalTest from the repo root)
# the motor's one-node winding temperature, Cth * dT/dt = P_copper - (T - T_coolant) / Rth:
# first the thermal model on its own, fed a fixed copper loss, against the datasheet's S1 rating,
# then on a dyno through the inverter, where the heat comes from the motor's own currents

CONTINUOUS_CURRENT_ARMS = 53.1    # datasheet "Io", continuous at stall, rated S1 dT = 100 K
DATASHEET_RISE_K = 100.0
PEAK_CURRENT_ARMS = 105.0         # datasheet "Imax", allowed for 1.24 s
PEAK_CURRENT_TIME_S = 1.24

DT_S = 5e-6    # motor electrical timestep on the dyno, controller runs once per switching period

# dyno: held at a fixed speed, the inverter told the continuous current (A peak, like a VESC's set-current)
DYNO_TEST = dict(
    speed_rpm=6000.0,
    bus_voltage_V=500.0,
    current_A=CONTINUOUS_CURRENT_ARMS * math.sqrt(2),
    duration_s=0.2,
)

def main():
    checkContinuousRating()
    checkTimeConstant()
    checkStepSize()
    checkCoolsDown()
    checkThermalOff()
    checkOnDyno()
    printPeakCurrent()
    print("All motor thermal checks passed")

# copper loss at a steady rms current: 3 phases x Rs x I^2 (the model's 1.5 * Rs * (id^2 + iq^2) in peak amps)
def getCopperLoss_W(motor, current_Arms):
    return 3 * motor.Rs * current_Arms ** 2

def getTimeConstant_s(motor):
    return motor.thermalResistance_KperW * motor.heatCapacity_JperK

# the datasheet's S1 rating: 53.1 A rms forever settles the windings 100 K over the coolant
def checkContinuousRating():
    motor = deepcopy(amk_dd5)
    copper_loss_W = getCopperLoss_W(motor, CONTINUOUS_CURRENT_ARMS)
    for _ in range(3600):    # an hour in 1 s steps, ~50 time constants
        motor.update_T_winding(copper_loss_W, 1.0)

    rise_K = motor.getWindingTemp_C() - motor.coolantTemp_C
    assert abs(rise_K - DATASHEET_RISE_K) < 1.0, \
        f"settled {rise_K:.1f} K over the coolant, the datasheet says {DATASHEET_RISE_K:.0f} K"
    print(f"{CONTINUOUS_CURRENT_ARMS} A rms continuous: {copper_loss_W:.0f} W copper loss, settles at "
          f"{motor.getWindingTemp_C():.1f} C (coolant + {rise_K:.1f} K, datasheet S1 dT = {DATASHEET_RISE_K:.0f} K)")

# after one time constant (Rth * Cth) it has done 63% of its rise, after three 95%
def checkTimeConstant():
    motor = deepcopy(amk_dd5)
    copper_loss_W = getCopperLoss_W(motor, CONTINUOUS_CURRENT_ARMS)
    final_rise_K = copper_loss_W * motor.thermalResistance_KperW
    tau_s = getTimeConstant_s(motor)

    step_s = 0.01
    for time_constants in (1, 2, 3):
        for _ in range(round(tau_s / step_s)):
            motor.update_T_winding(copper_loss_W, step_s)
        fraction = (motor.getWindingTemp_C() - motor.coolantTemp_C) / final_rise_K
        expected = 1 - math.exp(-time_constants)
        assert abs(fraction - expected) < 1e-3, \
            f"after {time_constants} time constants it's {fraction:.1%} of the way, expected {expected:.1%}"
        print(f"after {time_constants * tau_s:.0f} s ({time_constants} x Rth*Cth): {motor.getWindingTemp_C():.1f} C, "
              f"{fraction:.1%} of the rise")

# the result doesn't depend on the step size: one 1 s step lands where 200 000 of the motor's 5 us steps do
def checkStepSize():
    one_step = deepcopy(amk_dd5)
    small_steps = deepcopy(amk_dd5)
    copper_loss_W = getCopperLoss_W(one_step, PEAK_CURRENT_ARMS)

    one_step.update_T_winding(copper_loss_W, 1.0)
    for _ in range(round(1.0 / DT_S)):
        small_steps.update_T_winding(copper_loss_W, DT_S)

    difference_K = abs(one_step.getWindingTemp_C() - small_steps.getWindingTemp_C())
    assert difference_K < 1e-6, f"1 s in one step and in 5 us steps end up {difference_K:.2e} K apart"
    print(f"1 s at {PEAK_CURRENT_ARMS:.0f} A rms in one step or in 5 us steps: {one_step.getWindingTemp_C():.4f} C, "
          f"{difference_K:.0e} K apart")

# contactors open: no current, the coolant takes the heat back out. falls toward the coolant, never below it
def checkCoolsDown():
    motor = deepcopy(amk_dd5)
    motor.T_winding_C = motor.coolantTemp_C + DATASHEET_RISE_K    # as hot as a long continuous run leaves it
    tau_s = getTimeConstant_s(motor)

    temps_C = [motor.getWindingTemp_C()]
    for _ in range(round(5 * tau_s)):    # 5 time constants in 1 s steps
        motor.turnOff(1.0, wm=0.0)
        temps_C.append(motor.getWindingTemp_C())
    temps_C = np.array(temps_C)

    assert np.all(np.diff(temps_C) < 0), "the windings warmed up with no current"
    assert temps_C[-1] > motor.coolantTemp_C, "the windings cooled below the coolant"
    left = (temps_C[-1] - motor.coolantTemp_C) / DATASHEET_RISE_K
    assert abs(left - math.exp(-5)) < 1e-3, f"{left:.2%} of the rise left after 5 time constants, expected {math.exp(-5):.2%}"
    print(f"cool-down with the contactors open: {temps_C[0]:.0f} C -> {temps_C[-1]:.1f} C after {5 * tau_s:.0f} s")

# a motor config without Rth / Cth just stays at the coolant temperature
def checkThermalOff():
    motor = deepcopy(amk_dd5)
    motor.heatCapacity_JperK = None
    motor.update_T_winding(getCopperLoss_W(motor, PEAK_CURRENT_ARMS), 10.0)
    assert motor.getWindingTemp_C() == motor.coolantTemp_C, "heated up with no thermal values"
    print(f"no thermal values: stays at the coolant temperature ({motor.getWindingTemp_C():.0f} C)")

# the motor heats itself from its own copper loss, every 5 us step, driven through the inverter.
# over 0.2 s next to nothing reaches the coolant (time constant 70 s), so the rise is copper energy / heat capacity
def checkOnDyno():
    inverter = deepcopy(inverter1)
    motor = deepcopy(amk_dd5)
    control_period_s = inverter.switching_period_s
    steps_per_control_period = round(control_period_s / DT_S)
    speed_radps = DYNO_TEST["speed_rpm"] * 2 * np.pi / 60
    num_steps = round(DYNO_TEST["duration_s"] / DT_S)

    copper_loss_W = np.zeros(num_steps)
    for step in range(num_steps):
        if step % steps_per_control_period == 0:
            inverter.updateCurrentControl(None, motor, DYNO_TEST["bus_voltage_V"], control_period_s,
                                          current_request_A=DYNO_TEST["current_A"])
        motor.update_from_phase_voltages(inverter.getPhaseVoltages_V(), DT_S, wm=speed_radps)
        inverter.measureCurrents(motor.id, motor.iq)
        copper_loss_W[step] = motor.P_copper

    # once the current has settled the copper loss is the continuous rating's
    settled_loss_W = np.mean(copper_loss_W[num_steps // 2:])
    expected_loss_W = getCopperLoss_W(motor, CONTINUOUS_CURRENT_ARMS)
    assert abs(settled_loss_W - expected_loss_W) < 0.01 * expected_loss_W, \
        f"copper loss settled at {settled_loss_W:.0f} W, expected {expected_loss_W:.0f} W"

    rise_K = motor.getWindingTemp_C() - motor.coolantTemp_C
    expected_rise_K = np.sum(copper_loss_W) * DT_S / motor.heatCapacity_JperK
    assert abs(rise_K - expected_rise_K) < 0.01 * expected_rise_K, \
        f"rose {rise_K:.4f} K, the copper energy says {expected_rise_K:.4f} K"
    print(f"dyno, {CONTINUOUS_CURRENT_ARMS} A rms at {DYNO_TEST['speed_rpm']:.0f} rpm for {DYNO_TEST['duration_s']} s: "
          f"copper {settled_loss_W:.0f} W, +{rise_K:.4f} K (copper energy / heat capacity = {expected_rise_K:.4f} K)")

# not a check, a known limit: the datasheet only allows Imax for 1.24 s, but the one-node average barely moves
# in that time, so that limit protects a hotspot this model can't see (it isn't enforced anywhere yet)
def printPeakCurrent():
    motor = deepcopy(amk_dd5)
    copper_loss_W = getCopperLoss_W(motor, PEAK_CURRENT_ARMS)
    motor.update_T_winding(copper_loss_W, PEAK_CURRENT_TIME_S)
    print(f"note: {PEAK_CURRENT_ARMS:.0f} A rms for {PEAK_CURRENT_TIME_S} s (datasheet Imax): {copper_loss_W:.0f} W, "
          f"only +{motor.getWindingTemp_C() - motor.coolantTemp_C:.1f} K on the average winding temperature")

if __name__ == "__main__":
    main()
