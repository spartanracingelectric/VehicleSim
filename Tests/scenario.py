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
    vehicle: object = None                          # None = SR18

def stopWhenBmsShutsDown(vehicle):
    return vehicle.bms.isShutdown()

def runScenario(scenario):
    vehicle = startCar(scenario)
    control_period_s = vehicle.getControlPeriod_s()
    record_every = 1 if scenario.record_every_s is None else max(1, round(scenario.record_every_s / control_period_s))

    # one pass = one inverter switching period (100 us): the driver's pedal goes into the car
    log = []
    for tick in range(round(scenario.run_time_s / control_period_s)):
        time_s = tick * control_period_s
        apps_percent = scenario.pedal(time_s)
        vehicle.update(apps_percent)

        finished = isFinished(scenario, vehicle)
        if tick % record_every == 0 or finished:
            log.append(readCar(vehicle, tick, time_s, apps_percent))
        if finished:
            break

    # every reading becomes an array over the run: result.speed_mps[i] is the speed at result.time_s[i]
    result = SimpleNamespace(**{name: np.array([row[name] for row in log]) for name in log[0]})

    # numbers about the run and the car that the checks and plots need
    result.scenario_name = scenario.name
    result.control_period_s = control_period_s
    result.record_every = record_every
    result.end_time_s = result.time_s[-1] + control_period_s
    result.num_motors = vehicle.num_motors
    result.peak_torque_Nm = vehicle.motor.peakTorque_Nm
    result.max_torque_request_Nm = vehicle.vcu.max_torque_request_Nm
    result.vcu_loop_period_s = vehicle.vcu.loop_period_s
    result.bms_sample_period_s = vehicle.bms.sample_period_s
    result.energy_used_kWh = vehicle.battery.discharged_energy_kWh - vehicle.battery.charged_energy_kWh
    result.bms_faults = bmsFaults(vehicle.bms)
    return result

# a fresh copy of the car (so runs can't change each other), started the way the scenario says
def startCar(scenario):
    from Vehicle.MainConfigs.SR18 import SR18
    vehicle = deepcopy(scenario.vehicle if scenario.vehicle is not None else SR18)
    vehicle.battery.save_history = False    # the scenario keeps its own log
    if scenario.start_soc is not None or scenario.start_temp_C is not None:
        vehicle.battery.reset(initial_soc=scenario.start_soc, initial_temp_C=scenario.start_temp_C)
    vehicle.chassis.speed_mps = scenario.start_speed_kph / 3.6
    return vehicle

def isFinished(scenario, vehicle):
    if scenario.stop_at_distance_m is not None and vehicle.chassis.getDistance_m() >= scenario.stop_at_distance_m:
        return True
    return scenario.stop_when is not None and scenario.stop_when(vehicle)

# everything logged on one tick
def readCar(vehicle, tick, time_s, apps_percent):
    return {
        "tick": tick,
        "time_s": time_s,
        "apps_percent": float(apps_percent),
        "torque_request_Nm": vehicle.vcu.getTorqueRequest_Nm(),
        "torque_command_Nm": vehicle.inverter.getTorqueCommand_Nm(),
        "torque_feedback_Nm": vehicle.getTorqueFeedback_Nm(),
        "speed_mps": vehicle.chassis.getSpeed_mps(),
        "distance_m": vehicle.chassis.getDistance_m(),
        "motor_rpm": vehicle.getMotorSpeed_radps() * 60 / (2 * np.pi),
        "pack_power_W": vehicle.battery.terminal_power_W,
        "pack_current_A": vehicle.battery.current_A,
        "pack_voltage_V": vehicle.battery.terminal_voltage_V,
        "min_cell_voltage_mV": vehicle.bms.min_voltage_mV,
        "soc": vehicle.battery.soc,
        "pack_temp_C": vehicle.battery.temp_C,
        "motor_temp_C": vehicle.motor.getWindingTemp_C(),
        "bms_power_kW": vehicle.bms.power_kW,
        "power_threshold_kW": vehicle.bms.max_power_threshold_kW,
        "overpower": vehicle.bms.has_overpower_fault(),
        "shutdown": vehicle.bms.isShutdown(),
        "voltage_saturated": vehicle.inverter.isVoltageSaturated(),
        "traction_limited": vehicle.chassis.isTractionLimited(),
    }

# which faults are active at the end, for the summary
def bmsFaults(bms):
    faults = {
        "undervoltage": bms.has_undervoltage_fault(),
        "overvoltage": bms.has_overvoltage_fault(),
        "overtemperature": bms.has_overtemperature_fault(),
        "undertemperature": bms.has_undertemperature_fault(),
    }
    return [name for name, active in faults.items() if active]

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
            f"pack {result.pack_temp_C[-1]:.1f} C, motor {result.motor_temp_C[-1]:.1f} C")
    if result.overpower.any():
        text += f", overpower from {result.time_s[np.argmax(result.overpower)]:.2f} s"
    if result.shutdown.any():
        reason = ", ".join(result.bms_faults) or "latched earlier"
        text += f", bms shutdown at {result.time_s[np.argmax(result.shutdown)]:.2f} s ({reason})"
    return text
