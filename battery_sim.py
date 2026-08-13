import argparse
import math

from Vehicle.Battery.Configs.sr17 import sr17


POWER_KW = 50.0
DURATION_S = 60.0
DT_S = 0.1
INITIAL_SOC = 1.0
INITIAL_TEMP_C = 25.0
AMBIENT_TEMP_C = 25.0


class SimRunner:
    def __init__(
        self,
        battery,
        power_kw=POWER_KW,
        duration_s=DURATION_S,
        dt_s=DT_S,
        initial_soc=INITIAL_SOC,
        initial_temp_c=INITIAL_TEMP_C,
        ambient_temp_c=AMBIENT_TEMP_C,
    ):
        if duration_s <= 0 or dt_s <= 0:
            raise ValueError("Time and time step must be positive")
        if initial_soc < 0 or initial_soc > 1:
            raise ValueError("SOC must be between 0 and 1")

        self.battery = battery
        steps = math.ceil(duration_s / dt_s)
        self.time_s = [min(i * dt_s, duration_s) for i in range(steps + 1)]
        self.ambient_temp_C = ambient_temp_c

        n = len(self.time_s)
        self.requested_power_W = [power_kw * 1000] * n
        self.current_A = [0.0] * n
        self.open_circuit_voltage_V = [0.0] * n
        self.terminal_voltage_V = [0.0] * n
        self.terminal_power_W = [0.0] * n
        self.heat_W = [0.0] * n
        self.soc = [0.0] * n
        self.temp_C = [0.0] * n
        self.discharged_energy_kWh = [0.0] * n
        self.charged_energy_kWh = [0.0] * n
        self.shunt_coulomb_count_C = [0.0] * n

        self.soc[0] = initial_soc
        self.temp_C[0] = initial_temp_c


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--power-kw", type=float, default=POWER_KW)
    parser.add_argument("--duration-s", type=float, default=DURATION_S)
    parser.add_argument("--dt-s", type=float, default=DT_S)
    parser.add_argument("--initial-soc", type=float, default=INITIAL_SOC)
    parser.add_argument("--initial-temp-c", type=float, default=INITIAL_TEMP_C)
    parser.add_argument("--ambient-temp-c", type=float, default=AMBIENT_TEMP_C)
    args = parser.parse_args()

    sim = SimRunner(
        sr17,
        args.power_kw,
        args.duration_s,
        args.dt_s,
        args.initial_soc,
        args.initial_temp_c,
        args.ambient_temp_c,
    )
    run(sim)
    printResults(sim)


def run(sim):
    for i in range(len(sim.time_s) - 1):
        sim.battery.batteryCalc(i, sim)
    return sim


def printResults(sim):
    i = len(sim.time_s) - 2
    print(f"Configuration: {sim.battery.series_cells}s{sim.battery.parallel_cells}p")
    print(f"Nominal energy: {sim.battery.nominal_energy_kwh:.3f} kWh")
    print(f"Final SOC: {sim.soc[-1] * 100:.2f}%")
    print(f"Final pack voltage: {sim.terminal_voltage_V[i]:.2f} V")
    print(f"Final pack current: {sim.current_A[i]:.2f} A")
    print(f"Final temperature: {sim.temp_C[-1]:.2f} C")
    print(f"Energy discharged: {sim.discharged_energy_kWh[-1]:.3f} kWh")
    print(f"Energy charged: {sim.charged_energy_kWh[-1]:.3f} kWh")


if __name__ == "__main__":
    main()
