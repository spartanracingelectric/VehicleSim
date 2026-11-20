from copy import deepcopy
from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np

from Vehicle.Battery.Configs.sr17 import create_sr17_battery

# A long run of just the battery and the BMS: the power the driver asks for over time, with a power limit on top.
# The full car runs about 3x slower than real time and pack temperature only needs pack current, so this skips the drives.
# The demand keeps its length: on track a lower limit would make the same distance take longer.

@dataclass
class EnduranceRun:
    dt_s: float                     # 0.1 s or shorter: heat goes with current squared, so longer steps lose the peaks
    demand_power_W: object          # battery power asked for at each step (W). negative = regen
    start_soc: float = 1.0
    start_temp_C: float = 25.0
    battery: object = None          # None = the default sr17 pack


def runEndurance(run):
    battery = create_sr17_battery() if run.battery is None else deepcopy(run.battery)
    battery.save_history = False
    battery.reset(initial_soc=run.start_soc, initial_temp_C=run.start_temp_C)
    log = []
    for step, demand_W in enumerate(np.asarray(run.demand_power_W, dtype=float)):
        current_A = battery.currentForPower(demand_W, battery.getPackOpenCircuitVoltage_V())
        battery.update(current_A, run.dt_s)
        log.append((step * run.dt_s, battery.terminal_power_W, battery.current_A, battery.soc, battery.temp_C))
    names = ("time_s", "pack_power_W", "pack_current_A", "soc", "pack_temp_C")
    result = SimpleNamespace(**{name: np.array(values) for name, values in zip(names, zip(*log))})
    return result
