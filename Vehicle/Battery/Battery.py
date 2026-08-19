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
        *,
        initial_soc,
        initial_temp_C,
        ambient_temp_C,
    ):
        #TODO: 1. make cells object
        #TODO: 2. add comments to variables with weird ahh names
        #TODO: 3. change class name to tractive battery pack or TBP nomenclaturebs blame AKASH
        #TODO: 4. make calculation different functions cuz im a dickhead
        #TODO: 5. make arrays into numpy arrays and use numpy mtfker
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
        self.initial_soc = initial_soc
        self.initial_temp_C = initial_temp_C
        self.ambient_temp_C = ambient_temp_C

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

        self.reset()

    def reset(self, initial_soc=None, initial_temp_C=None, ambient_temp_C=None):
        if initial_soc is None:
            initial_soc = self.initial_soc
        if initial_temp_C is None:
            initial_temp_C = self.initial_temp_C
        if ambient_temp_C is None:
            ambient_temp_C = self.ambient_temp_C

        if initial_soc < 0 or initial_soc > 1:
            raise ValueError("SOC must be between 0 and 1")

        self.soc = initial_soc
        self.temp_C = initial_temp_C
        self.ambient_temp_C = ambient_temp_C
        self.elapsed_time_s = 0.0
        self.current_A = 0.0
        self.open_circuit_voltage_V = self.interpolate(self.soc) * self.series_cells
        self.terminal_voltage_V = self.open_circuit_voltage_V
        self.terminal_power_W = 0.0
        self.heat_W = 0.0
        self.discharged_energy_kWh = 0.0
        self.charged_energy_kWh = 0.0
        self.shunt_coulomb_count_C = 0.0

        self.time_s = [0.0]
        self.soc_history = [self.soc]
        self.temp_history_C = [self.temp_C]
        self.current_history_A = [self.current_A]
        self.voltage_history_V = [self.terminal_voltage_V]
        self.power_history_W = [self.terminal_power_W]

    def update(self, current_A, dt):
        if dt <= 0:
            raise ValueError("Time step must be positive")

        self.open_circuit_voltage_V = (
            self.interpolate(self.soc) * self.series_cells
        )
        # Current comes from whatever is drawing from or charging the battery.
        # self.current_A = self.currentForPower(
        #     power_W, self.open_circuit_voltage_V
        # )
        self.current_A = current_A
        self.terminal_voltage_V = (
            self.open_circuit_voltage_V
            - self.current_A * self.internal_resistance_ohm
        )
        self.terminal_power_W = self.terminal_voltage_V * self.current_A
        self.heat_W = self.current_A**2 * self.internal_resistance_ohm

        used_ah = self.current_A * dt / 3600
        next_soc = self.soc - used_ah / self.capacity_ah
        if next_soc < 0 or next_soc > 1:
            raise ValueError("Battery SOC went outside the modeled range")
        self.soc = next_soc

        self.temp_C = self.temperatureCalc(self.heat_W, dt)

        energy = self.terminal_power_W * dt / 3_600_000
        self.discharged_energy_kWh += max(energy, 0)
        self.charged_energy_kWh += max(-energy, 0)
        self.shunt_coulomb_count_C += self.current_A * dt
        self.elapsed_time_s += dt

        self.time_s.append(self.elapsed_time_s)
        self.soc_history.append(self.soc)
        self.temp_history_C.append(self.temp_C)
        self.current_history_A.append(self.current_A)
        self.voltage_history_V.append(self.terminal_voltage_V)
        self.power_history_W.append(self.terminal_power_W)

    def currentForPower(self, power_W, voltage_V=None):
        if voltage_V is None:
            voltage_V = self.open_circuit_voltage_V

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

    def temperatureCalc(self, heat_W, dt):
        if self.thermal_resistance_kpw is None:
            return self.temp_C + heat_W * dt / self.heat_capacity_jpk

        final_temp = (
            self.ambient_temp_C + heat_W * self.thermal_resistance_kpw
        )
        time_constant = self.thermal_resistance_kpw * self.heat_capacity_jpk
        return final_temp + (self.temp_C - final_temp) * math.exp(
            -dt / time_constant
        )

    def getPackMaxVoltage_V(self):
        return self.interpolate(self.soc) * self.series_cells

    def getPackMaxCurrent_A(self):
        max_voltage_V = self.getPackMaxVoltage_V()
        voltage_limited_current_A = (
            (max_voltage_V - self.min_voltage_v)
            / self.internal_resistance_ohm
        )
        return min(
            self.max_discharge_current_a,
            max(0.0, voltage_limited_current_A),
        )

    # Battery sensor values used by the BMS model.
    def getCellVoltages_mV(self):
        cell_voltage_V = self.terminal_voltage_V / self.series_cells
        voltages = []
        for cell in range(self.series_cells):
            voltage_mV = cell_voltage_V * 1000
            voltage_mV += self.cell_voltage_offsets_mV[cell]
            voltages.append(round(voltage_mV))
        return voltages

    def getTemperatures_C(self):
        temperatures = []
        for thermistor in range(len(self.temperature_offsets_C)):
            temperature = self.temp_C + self.temperature_offsets_C[thermistor]
            temperatures.append(int(temperature))
        return temperatures

    def getModuleData(self):
        cell_voltages = self.getCellVoltages_mV()
        temperatures = self.getTemperatures_C()
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

    def getHvSensePackVoltage_cV(self):
        voltage_V = self.terminal_voltage_V + self.hv_sense_offset_V
        return round(voltage_V * 100)

    def getShuntCurrent_mA(self):
        return round(self.current_A * 1000)

    def getBMSInputs(self):
        modules = self.getModuleData()
        cell_voltages = []
        for module in modules:
            cell_voltages.extend(module["cellVoltage_mV"])

        return {
            "moduleData": modules,
            "sumPackVoltage_cV": round(sum(cell_voltages) / 10),
            "hvSensePackVoltage_cV": self.getHvSensePackVoltage_cV(),
            "shuntCurrent_mA": self.getShuntCurrent_mA(),
            "shuntCoulombCount_C": self.shunt_coulomb_count_C,
        }
