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
        num_modules=10,
        cells_per_module=14,
        thermistors_per_module=10,
        cell_voltage_offsets_mV=None,
        temperature_offsets_C=None,
        hv_sense_offset_V=0.0,
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
        self.num_modules = num_modules
        self.cells_per_module = cells_per_module
        self.thermistors_per_module = thermistors_per_module
        self.hv_sense_offset_V = hv_sense_offset_V

        if num_modules * cells_per_module != series_cells:
            raise ValueError("Module cell count does not match series cell count")

        if cell_voltage_offsets_mV is None:
            cell_voltage_offsets_mV = [0.0] * series_cells
        if temperature_offsets_C is None:
            temperature_offsets_C = [0.0] * (
                num_modules * thermistors_per_module
            )
        self.cell_voltage_offsets_mV = cell_voltage_offsets_mV
        self.temperature_offsets_C = temperature_offsets_C

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
        sim.shunt_coulomb_count_C[i + 1] = (
            sim.shunt_coulomb_count_C[i] + sim.current_A[i] * dt
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

    # BMS stuff.
    def getCellVoltages_mV(self, sim, i=None):
        if i is None:
            i = len(sim.time_s) - 2

        cell_voltage_V = sim.terminal_voltage_V[i] / self.series_cells
        voltages = []
        for cell in range(self.series_cells):
            voltage_mV = cell_voltage_V * 1000
            voltage_mV += self.cell_voltage_offsets_mV[cell]
            voltages.append(round(voltage_mV))
        return voltages

    def getTemperatures_C(self, sim, i=None):
        if i is None:
            i = len(sim.time_s) - 2

        temperatures = []
        temp_C = sim.temp_C[i + 1]
        for thermistor in range(len(self.temperature_offsets_C)):
            temperature = temp_C + self.temperature_offsets_C[thermistor]
            temperatures.append(int(temperature))
        return temperatures

    def getModuleData(self, sim, i=None):
        cell_voltages = self.getCellVoltages_mV(sim, i)
        temperatures = self.getTemperatures_C(sim, i)
        modules = []

        for module in range(self.num_modules):
            first_cell = module * self.cells_per_module
            last_cell = first_cell + self.cells_per_module
            first_therm = module * self.thermistors_per_module
            last_therm = first_therm + self.thermistors_per_module
            modules.append({
                "cellVoltage_mV": cell_voltages[first_cell:last_cell],
                "pointTemp_C": temperatures[first_therm:last_therm],
            })
        return modules

    def getHvSensePackVoltage_cV(self, sim, i=None):
        if i is None:
            i = len(sim.time_s) - 2
        voltage_V = sim.terminal_voltage_V[i] + self.hv_sense_offset_V
        return round(voltage_V * 100)

    def getShuntCurrent_mA(self, sim, i=None):
        if i is None:
            i = len(sim.time_s) - 2
        return round(sim.current_A[i] * 1000)

    def getBMSInputs(self, sim, i=None):
        if i is None:
            i = len(sim.time_s) - 2

        modules = self.getModuleData(sim, i)
        cell_voltages = []
        for module in modules:
            cell_voltages.extend(module["cellVoltage_mV"])

        return {
            "moduleData": modules,
            "sumPackVoltage_cV": round(sum(cell_voltages) / 10),
            "hvSensePackVoltage_cV": self.getHvSensePackVoltage_cV(sim, i),
            "shuntCurrent_mA": self.getShuntCurrent_mA(sim, i),
            "shuntCoulombCount_C": sim.shunt_coulomb_count_C[i + 1],
        }
