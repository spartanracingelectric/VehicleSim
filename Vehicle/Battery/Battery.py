import math


class Battery:
    def __init__(
        self,
        mass_kg,
        series_cells=140,
        parallel_cells=3,
        cell_capacity_ah=5.0,
        cell_nominal_voltage_v=3.6,
        cell_internal_resistance_ohm=6.7e-3,
        cell_max_discharge_current_a=125.0,
        specific_heat_jpkgk=900.0,
        thermal_resistance_kpw=None,
        ocv_soc=None,
        ocv_cell_voltage_v=None,
    ):
        self.mass_kg = mass_kg
        self.series_cells = series_cells
        self.parallel_cells = parallel_cells
        self.cell_capacity_ah = cell_capacity_ah
        self.cell_nominal_voltage_v = cell_nominal_voltage_v
        self.cell_internal_resistance_ohm = cell_internal_resistance_ohm
        self.cell_max_discharge_current_a = cell_max_discharge_current_a
        self.specific_heat_jpkgk = specific_heat_jpkgk
        self.thermal_resistance_kpw = thermal_resistance_kpw

        self.total_cells = series_cells * parallel_cells
        self.capacity_ah = parallel_cells * cell_capacity_ah
        self.nominal_voltage_v = series_cells * cell_nominal_voltage_v
        self.nominal_energy_kwh = self.nominal_voltage_v * self.capacity_ah / 1000
        self.internal_resistance_ohm = (
            series_cells * cell_internal_resistance_ohm / parallel_cells
        )
        self.max_discharge_current_a = (
            parallel_cells * cell_max_discharge_current_a
        )
        self.heat_capacity_jpk = mass_kg * specific_heat_jpkgk

        if ocv_soc is None:
            ocv_soc = [0.00, 0.20, 0.50, 0.80, 1.00]
        if ocv_cell_voltage_v is None:
            ocv_cell_voltage_v = [2.50, 3.55, 3.70, 3.88, 4.20]

        self.ocv_soc = ocv_soc
        self.ocv_cell_voltage_v = ocv_cell_voltage_v
        self.min_voltage_v = series_cells * self.ocv_cell_voltage_v[0]
        self.max_voltage_v = series_cells * self.ocv_cell_voltage_v[-1]

    def batteryCalc(self, i, sim):
        dt = sim.time_s[i + 1] - sim.time_s[i]
        cell_voltage = self.interpolate(sim.soc[i])
        sim.open_circuit_voltage_V[i] = cell_voltage * self.series_cells

        sim.current_A[i] = self.currentForPower(
            sim.requested_power_W[i], sim.open_circuit_voltage_V[i]
        )
        if sim.current_A[i] > self.max_discharge_current_a:
            raise ValueError("Battery current limit exceeded")

        sim.terminal_voltage_V[i] = (
            sim.open_circuit_voltage_V[i]
            - sim.current_A[i] * self.internal_resistance_ohm
        )
        sim.terminal_power_W[i] = sim.terminal_voltage_V[i] * sim.current_A[i]
        sim.heat_W[i] = sim.current_A[i] ** 2 * self.internal_resistance_ohm

        used_ah = sim.current_A[i] * dt / 3600
        sim.soc[i + 1] = sim.soc[i] - used_ah / self.capacity_ah
        if sim.soc[i + 1] < 0 or sim.soc[i + 1] > 1:
            raise ValueError("Battery ran out of charge")

        sim.temp_C[i + 1] = self.temperatureCalc(
            sim.temp_C[i], sim.heat_W[i], dt, sim.ambient_temp_C
        )

        energy = sim.terminal_power_W[i] * dt / 3_600_000
        sim.discharged_energy_kWh[i + 1] = (
            sim.discharged_energy_kWh[i] + max(energy, 0)
        )
        sim.charged_energy_kWh[i + 1] = (
            sim.charged_energy_kWh[i] + max(-energy, 0)
        )

    def currentForPower(self, power_W, voltage_V):
        d = voltage_V**2 - 4 * self.internal_resistance_ohm * power_W
        if d < 0:
            raise ValueError("Battery cannot supply that much power")
        return 2 * power_W / (voltage_V + math.sqrt(d))

    def interpolate(self, soc):
        for i in range(len(self.ocv_soc) - 1):
            if soc <= self.ocv_soc[i + 1]:
                amount = (
                    (soc - self.ocv_soc[i])
                    / (self.ocv_soc[i + 1] - self.ocv_soc[i])
                )
                return self.ocv_cell_voltage_v[i] + amount * (
                    self.ocv_cell_voltage_v[i + 1]
                    - self.ocv_cell_voltage_v[i]
                )
        return self.ocv_cell_voltage_v[-1]

    def temperatureCalc(self, temp_C, heat_W, dt, ambient_C):
        if self.thermal_resistance_kpw is None:
            return temp_C + heat_W * dt / self.heat_capacity_jpk

        final_temp = ambient_C + heat_W * self.thermal_resistance_kpw
        time_constant = self.thermal_resistance_kpw * self.heat_capacity_jpk
        return final_temp + (temp_C - final_temp) * math.exp(
            -dt / time_constant
        )
