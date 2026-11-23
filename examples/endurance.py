"""Endurance on the battery alone: what a power limit does to pack temperature.

    python -m examples.endurance

The demand is a made-up lap, not a log: swap in your own battery power (W, one value per 0.1 s) to use real data.
"""
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from Vehicle.Simulation.endurance import EnduranceRun, runEndurance, solvePowerLimit_W

TARGET_C = 58.0
# one stretch between corners as (seconds, battery kW), about 14 kW on average, repeated for 27.5 minutes
STRETCH = [(2.5, 40.0), (1.5, -6.0), (3.0, 6.0), (2.0, 28.0), (1.0, -4.0), (2.5, 4.0)]
DEMAND_W = np.tile(np.concatenate([np.full(round(seconds / 0.1), kW * 1000.0) for seconds, kW in STRETCH]), 132)


def main():
    limit_W = solvePowerLimit_W(EnduranceRun(0.1, DEMAND_W, start_temp_C=30.0), TARGET_C)
    for label, power_limit_W in (("no limit", None), (f"{limit_W / 1000:.1f} kW limit", limit_W)):
        result = runEndurance(EnduranceRun(0.1, DEMAND_W, power_limit_W, start_temp_C=30.0))
        print(f"{label}: peak {result.peak_temp_C:.1f} C, used {result.energy_used_kWh:.2f} kWh"
              + (f", bms shutdown at {result.time_s[-1]:.0f} s" if result.shutdown else ""))
        plt.plot(result.time_s / 60, result.pack_temp_C, label=label)
    plt.axhline(TARGET_C, color="gray", linestyle="--")
    plt.xlabel("Time (min)")
    plt.ylabel("Thermistor temperature (C)")
    plt.legend()
    save_path = Path(__file__).resolve().parents[1] / "outputs/endurance.png"
    save_path.parent.mkdir(exist_ok=True)
    plt.savefig(save_path, dpi=160)
    print(f"Saved graph to {save_path}")
    plt.show()


if __name__ == "__main__":
    main()
