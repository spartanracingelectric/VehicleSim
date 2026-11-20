import math
from copy import deepcopy
from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np

from Vehicle.Battery.Configs.sr17 import create_sr17_battery
from Vehicle.BMS.Configs.bms1 import bms1

# A long run of just the battery and the BMS: the power the driver asks for over time, with a power limit on top.
# The full car runs about 3x slower than real time and pack temperature only needs pack current, so this skips the drives.
# The demand keeps its length: on track a lower limit would make the same distance take longer.

@dataclass
class EnduranceRun:
    dt_s: float                     # 0.1 s or shorter: heat goes with current squared, so longer steps lose the peaks
    demand_power_W: object          # battery power asked for at each step (W). negative = regen
    power_limit_W: object = None    # None = no limit, a number (W), or f(time_s, battery, bms) -> W
    start_soc: float = 1.0
    start_temp_C: float = 25.0
    battery: object = None          # None = the default sr17 pack


def runEndurance(run):
    battery = create_sr17_battery() if run.battery is None else deepcopy(run.battery)
    bms = deepcopy(bms1)
    battery.save_history = False
    battery.reset(initial_soc=run.start_soc, initial_temp_C=run.start_temp_C)
    bms.reset_shutdown()
    # the bms reads the pack before the run, so a pack that starts too hot never closes its contactors
    bms.update(battery.getCellVoltages_mV(), battery.getTemperatures_C(), battery.getShuntCurrent_mA())

    log = []
    for step, demand_W in enumerate(np.asarray(run.demand_power_W, dtype=float)):
        time_s = step * run.dt_s
        limit_W = run.power_limit_W(time_s, battery, bms) if callable(run.power_limit_W) else run.power_limit_W
        current_A = 0.0
        if not bms.isShutdown():
            # only discharge is limited, as in the vcu: regen passes through
            power_W = max(0.0, min(demand_W, math.inf if limit_W is None else limit_W)) if demand_W > 0 else demand_W
            current_A = battery.currentForPower(power_W, battery.getPackOpenCircuitVoltage_V())
            if not 0.0 <= battery.getNextSoc(current_A, run.dt_s) <= 1.0:    # a full pack takes no regen
                current_A = 0.0
        battery.update(current_A, run.dt_s)
        bms.updateFromPack(battery, run.dt_s)
        log.append((time_s, battery.terminal_power_W, battery.current_A, battery.soc, battery.temp_C,
                    bms.max_temperature_C))
        if bms.isShutdown():
            break

    names = ("time_s", "pack_power_W", "pack_current_A", "soc", "pack_temp_C", "bms_max_temp_C")
    result = SimpleNamespace(**{name: np.array(values) for name, values in zip(names, zip(*log))})
    result.peak_temp_C = float(max(run.start_temp_C, result.pack_temp_C.max()))
    result.energy_used_kWh = battery.discharged_energy_kWh - battery.charged_energy_kWh
    result.shutdown = bms.isShutdown()
    return result
