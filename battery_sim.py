import argparse
import math

from Vehicle.Battery.Configs.sr17 import sr17


POWER_KW = 5.0
DURATION_S = 600.0
DT_S = 0.1
FAST_INTEGRATION_STEPS = 32
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
        record_history=False,
    ):
        if duration_s <= 0 or dt_s <= 0:
            raise ValueError("Time and time step must be positive")
        if initial_soc < 0 or initial_soc > 1:
            raise ValueError("SOC must be between 0 and 1")

        self.battery = battery
        self.duration_s = duration_s
        self.dt_s = dt_s
        self.power_W = power_kw * 1000
        self.initial_soc = initial_soc
        self.initial_temp_C = initial_temp_c
        self.ambient_temp_C = ambient_temp_c
        self.record_history = record_history

        if record_history:
            steps = math.ceil(duration_s / dt_s)
            self.time_s = [min(i * dt_s, duration_s) for i in range(steps + 1)]
        else:
            self.time_s = [0.0, duration_s]

        n = len(self.time_s)
        self.requested_power_W = [self.power_W] * n
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
    parser.add_argument(
        "--record-history",
        action="store_true",
        help="save every dt sample",
    )
    args = parser.parse_args()

    sim = SimRunner(
        sr17,
        args.power_kw,
        args.duration_s,
        args.dt_s,
        args.initial_soc,
        args.initial_temp_c,
        args.ambient_temp_c,
        args.record_history,
    )
    run(sim)
    printResults(sim)


def run(sim):
    if sim.record_history:
        return _run_with_history(sim)
    _run_fast(sim)
    return sim


def _derivatives(sim, soc, temp_C):
    battery = sim.battery
    voltage_V = battery.interpolate(soc) * battery.series_cells
    current_A = battery.currentForPower(sim.power_W, voltage_V)
    heat_W = current_A * current_A * battery.internal_resistance_ohm
    dsoc_dt = -current_A / (3600 * battery.capacity_ah)

    if battery.thermal_resistance_kpw is None:
        dtemp_dt = heat_W / battery.heat_capacity_jpk
    else:
        cooling_W = (temp_C - sim.ambient_temp_C) / battery.thermal_resistance_kpw
        dtemp_dt = (heat_W - cooling_W) / battery.heat_capacity_jpk
    return dsoc_dt, dtemp_dt


def _run_fast(sim):
    battery = sim.battery
    soc = sim.initial_soc
    temp_C = sim.initial_temp_C
    h = sim.duration_s / FAST_INTEGRATION_STEPS

    for _ in range(FAST_INTEGRATION_STEPS):
        s1, t1 = _derivatives(sim, soc, temp_C)
        s2, t2 = _derivatives(sim, soc + 0.5 * h * s1, temp_C + 0.5 * h * t1)
        s3, t3 = _derivatives(sim, soc + 0.5 * h * s2, temp_C + 0.5 * h * t2)
        s4, t4 = _derivatives(sim, soc + h * s3, temp_C + h * t3)
        soc += h * (s1 + 2 * s2 + 2 * s3 + s4) / 6
        temp_C += h * (t1 + 2 * t2 + 2 * t3 + t4) / 6
        if not 0 <= soc <= 1:
            raise ValueError("Pack SOC went outside the modeled range")

    ocv_V = battery.interpolate(soc) * battery.series_cells
    current_A = battery.currentForPower(sim.power_W, ocv_V)
    terminal_voltage_V = ocv_V - current_A * battery.internal_resistance_ohm
    heat_W = current_A * current_A * battery.internal_resistance_ohm
    energy_kWh = sim.power_W * sim.duration_s / 3_600_000
    coulombs = (sim.initial_soc - soc) * battery.capacity_ah * 3600

    sim.soc[:] = [sim.initial_soc, soc]
    sim.temp_C[:] = [sim.initial_temp_C, temp_C]
    sim.current_A[:] = [0.0, current_A]
    sim.open_circuit_voltage_V[:] = [
        battery.interpolate(sim.initial_soc) * battery.series_cells,
        ocv_V,
    ]
    sim.terminal_voltage_V[:] = [sim.open_circuit_voltage_V[0], terminal_voltage_V]
    sim.terminal_power_W[:] = [0.0, sim.power_W]
    sim.heat_W[:] = [0.0, heat_W]
    sim.discharged_energy_kWh[:] = [0.0, max(energy_kWh, 0.0)]
    sim.charged_energy_kWh[:] = [0.0, max(-energy_kWh, 0.0)]
    sim.shunt_coulomb_count_C[:] = [0.0, coulombs]

    battery.reset(sim.initial_soc, sim.initial_temp_C, sim.ambient_temp_C)
    battery.soc = soc
    battery.temp_C = temp_C
    battery.elapsed_time_s = sim.duration_s
    battery.current_A = current_A
    battery.open_circuit_voltage_V = ocv_V
    battery.terminal_voltage_V = terminal_voltage_V
    battery.terminal_power_W = sim.power_W
    battery.heat_W = heat_W
    battery.discharged_energy_kWh = max(energy_kWh, 0.0)
    battery.charged_energy_kWh = max(-energy_kWh, 0.0)
    battery.shunt_coulomb_count_C = coulombs
    battery.time_s.append(sim.duration_s)
    battery.soc_history.append(soc)
    battery.temp_history_C.append(temp_C)
    battery.current_history_A.append(current_A)
    battery.voltage_history_V.append(terminal_voltage_V)
    battery.power_history_W.append(sim.power_W)


def _run_with_history(sim):
    battery = sim.battery
    battery.reset(sim.initial_soc, sim.initial_temp_C, sim.ambient_temp_C)
    discharged_energy_kWh = [0.0]
    charged_energy_kWh = [0.0]
    coulomb_count_C = [0.0]
    for i in range(len(sim.time_s) - 1):
        dt = sim.time_s[i + 1] - sim.time_s[i]
        voltage_V = battery.interpolate(battery.soc) * battery.series_cells
        current_A = battery.currentForPower(sim.power_W, voltage_V)
        battery.update(current_A, dt)
        discharged_energy_kWh.append(battery.discharged_energy_kWh)
        charged_energy_kWh.append(battery.charged_energy_kWh)
        coulomb_count_C.append(battery.shunt_coulomb_count_C)

    sim.soc[:] = battery.soc_history
    sim.temp_C[:] = battery.temp_history_C
    sim.current_A[:] = battery.current_history_A
    sim.terminal_voltage_V[:] = battery.voltage_history_V
    sim.open_circuit_voltage_V[:] = [
        voltage + current * battery.internal_resistance_ohm
        for voltage, current in zip(sim.terminal_voltage_V, sim.current_A)
    ]
    sim.terminal_power_W[:] = battery.power_history_W
    sim.heat_W[:] = [
        current * current * battery.internal_resistance_ohm
        for current in sim.current_A
    ]
    sim.discharged_energy_kWh[:] = discharged_energy_kWh
    sim.charged_energy_kWh[:] = charged_energy_kWh
    sim.shunt_coulomb_count_C[:] = coulomb_count_C
    return sim


def printResults(sim):
    i = len(sim.time_s) - 1
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
