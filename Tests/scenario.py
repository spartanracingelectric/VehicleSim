import numpy as np
from copy import deepcopy
from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Callable

# One straight-line run of the car: who's driving (pedal over time), how the car starts, when it stops,
# and what has to be true afterwards. runScenario does the loop so a test only has to say what's different.

@dataclass
class Scenario:
    name: str
    pedal: Callable[[float], float]                 # time (s) -> pedal %, see Tests/pedals.py
    checks: list = field(default_factory=list)      # rules for the result, see Tests/checks.py
    run_time_s: float = 5.0                         # stops here at the latest
    stop_at_distance_m: float = None                # or as soon as the car has gone this far
    stop_when: Callable = None                      # or as soon as stop_when(vehicle) is True, e.g. stopWhenBmsShutsDown
    start_soc: float = None                         # None = whatever the car's config starts at
    start_temp_C: float = None
    start_speed_kph: float = 0.0
    record_every_s: float = None                    # None = log every tick (100 us), long runs can log less often
    vehicle: object = None                          # None = SR17

def stopWhenBmsShutsDown(vehicle):
    return vehicle.bms.isShutdown()

# everything recorded every logged tick
CHANNELS = ("apps_percent", "torque_request_Nm", "torque_command_Nm", "torque_feedback_Nm", "speed_mps",
            "distance_m", "motor_rpm", "pack_power_W", "pack_voltage_V", "pack_current_A",
            "min_cell_voltage_mV", "soc", "pack_temp_C", "bms_power_kW")
FLAGS = ("overpower", "shutdown", "voltage_saturated", "traction_limited")

def runScenario(scenario):
    from Vehicle.MainConfigs.SR17 import SR17
    vehicle = deepcopy(scenario.vehicle if scenario.vehicle is not None else SR17)
    vehicle.battery.save_history = False    # the scenario keeps its own log below

    # starting conditions
    if scenario.start_soc is not None or scenario.start_temp_C is not None:
        vehicle.battery.reset(initial_soc=scenario.start_soc, initial_temp_C=scenario.start_temp_C)
    vehicle.speed_mps = scenario.start_speed_kph / 3.6

    control_period_s = vehicle.getControlPeriod_s()
    max_periods = round(scenario.run_time_s / control_period_s)
    record_every = 1 if scenario.record_every_s is None else max(1, round(scenario.record_every_s / control_period_s))
    max_records = (max_periods - 1) // record_every + 2     # +1 for the last tick if it stops early
    result = SimpleNamespace(
        scenario_name=scenario.name,
        num_motors=vehicle.num_motors,
        peak_torque_Nm=vehicle.motor.peakTorque_Nm,
        max_torque_request_Nm=vehicle.vcu.max_torque_request_Nm,
        vcu_loop_period_s=vehicle.vcu.loop_period_s,
        bms_sample_period_s=vehicle.bms.sample_period_s,
        control_period_s=control_period_s,
        record_every=record_every,
        tick=np.zeros(max_records, dtype=int),          # which tick each logged row is
        time_s=np.zeros(max_records),
        power_threshold_kW=np.full(max_records, vehicle.bms.max_power_threshold_kW),
    )

    # This for loop does -> result."variable_name" = np.zeros(num_periods)
    for channel in CHANNELS:
        setattr(result, channel, np.zeros(max_records))

    # This for loop does -> result."variable_name" = np.zeroes(num_periods, dtype = bool)
    for flag in FLAGS:
        setattr(result, flag, np.zeros(max_records, dtype=bool))

    # one pass = one inverter switching period (100 us): the driver's pedal goes into the car
    row = 0
    for k in range(max_periods):
        time_s = k * control_period_s
        apps_percent = scenario.pedal(time_s)
        vehicle.update(apps_percent)

        stopping = ((scenario.stop_at_distance_m is not None and vehicle.getDistance_m() >= scenario.stop_at_distance_m)
                    or (scenario.stop_when is not None and scenario.stop_when(vehicle)))
        if k % record_every == 0 or stopping:
            result.tick[row] = k
            result.time_s[row] = time_s
            record(result, row, vehicle, apps_percent)
            row += 1
        if stopping:
            break

    # cut everything down to how far it actually ran
    for name in ("tick", "time_s", "power_threshold_kW") + CHANNELS + FLAGS:
        setattr(result, name, getattr(result, name)[:row])

    result.end_time_s = result.time_s[-1] + control_period_s
    result.energy_used_kWh = vehicle.battery.discharged_energy_kWh - vehicle.battery.charged_energy_kWh
    result.bms_faults = [name for name, active in (("undervoltage", vehicle.bms.has_undervoltage_fault()),
                                                   ("overvoltage", vehicle.bms.has_overvoltage_fault()),
                                                   ("overtemperature", vehicle.bms.has_overtemperature_fault()),
                                                   ("undertemperature", vehicle.bms.has_undertemperature_fault()))
                         if active]
    return result

# Collects the results for every loop
def record(result, row, vehicle, apps_percent):
    result.apps_percent[row] = apps_percent
    result.torque_request_Nm[row] = vehicle.vcu.getTorqueRequest_Nm()
    result.torque_command_Nm[row] = vehicle.inverter.getTorqueCommand_Nm()
    result.torque_feedback_Nm[row] = vehicle.getTorqueFeedback_Nm()
    result.speed_mps[row] = vehicle.getSpeed_mps()
    result.distance_m[row] = vehicle.getDistance_m()
    result.motor_rpm[row] = vehicle.getMotorSpeed_radps() * 60 / (2 * np.pi)
    result.pack_power_W[row] = vehicle.battery.terminal_power_W
    result.pack_current_A[row] = vehicle.battery.current_A
    result.pack_voltage_V[row] = vehicle.battery.terminal_voltage_V
    result.min_cell_voltage_mV[row] = vehicle.bms.min_voltage_mV
    result.soc[row] = vehicle.battery.soc
    result.pack_temp_C[row] = vehicle.battery.temp_C
    result.bms_power_kW[row] = vehicle.bms.power_kW
    result.overpower[row] = vehicle.bms.has_overpower_fault()
    result.shutdown[row] = vehicle.bms.isShutdown()
    result.voltage_saturated[row] = vehicle.inverter.isVoltageSaturated()
    result.traction_limited[row] = vehicle.isTractionLimited()

# pedal and torque, speed and motor rpm, pack power vs the 80 kW flag, pack voltage and current
# save_path=None opens a window, otherwise saves a png there
def plotScenario(result, save_path=None):
    from Data.plot import plotAcceleration
    plotAcceleration(
        result.time_s,
        result.apps_percent,
        result.torque_request_Nm,
        result.torque_feedback_Nm,
        result.speed_mps * 3.6,
        result.motor_rpm,
        result.pack_power_W / 1000,
        result.power_threshold_kW,
        result.overpower,
        result.pack_voltage_V,
        result.pack_current_A,
        result.scenario_name,
        save_path,
    )

# one line about how a run went
def summarize(result):
    text = (f"{result.scenario_name}: {result.end_time_s:.2f} s, {result.speed_mps[-1] * 3.6:.0f} km/h, "
            f"{result.distance_m[-1]:.1f} m, peak {np.max(result.pack_power_W) / 1000:.1f} kW, "
            f"used {result.energy_used_kWh:.3f} kWh, SOC {result.soc[0]:.1%} -> {result.soc[-1]:.1%}, "
            f"pack {result.pack_temp_C[-1]:.1f} C")
    if result.overpower.any():
        text += f", overpower from {result.time_s[np.argmax(result.overpower)]:.2f} s"
    if result.shutdown.any():
        reason = ", ".join(result.bms_faults) or "latched earlier"
        text += f", bms shutdown at {result.time_s[np.argmax(result.shutdown)]:.2f} s ({reason})"
    return text
