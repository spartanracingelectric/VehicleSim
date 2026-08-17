import math

from Vehicle.Battery.Cell import Cell


class Pack:
    def __init__(
        self,
        mass_kg,
        cell=None,
        series_cells=140,
        parallel_cells=3,
        specific_heat_jpkgk=900.0,    # joules per kilogram kelvin
        thermal_resistance_kpw=None,  # kelvin per watt
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
        # cell used in the pack
        if cell is None:
            cell = Cell()
        self.cell = cell

        # pack size
        self.mass_kg = mass_kg
        self.series_cells = series_cells
        self.parallel_cells = parallel_cells

        # how hard the pack is to heat up
        self.specific_heat_jpkgk = specific_heat_jpkgk

        # how fast heat leaves the pack
        self.thermal_resistance_kpw = thermal_resistance_kpw

        # bms module layout
        self.num_modules = num_modules
        self.cells_per_module = cells_per_module
        self.thermistors_per_module = thermistors_per_module

        # fake error for the pack voltage sensor
        self.hv_sense_offset_V = hv_sense_offset_V

        # starting values
        self.initial_soc = initial_soc
        self.initial_temp_C = initial_temp_C
        self.ambient_temp_C = ambient_temp_C

        if num_modules * cells_per_module != series_cells:
            raise ValueError("Module cell count does not match series cell count")

        # fake error for each sensor
        if cell_voltage_offsets_mV is None:
            cell_voltage_offsets_mV = [0.0] * series_cells
        if temperature_offsets_C is None:
            temperature_offsets_C = [0.0] * (
                num_modules * thermistors_per_module
            )
        if len(cell_voltage_offsets_mV) != series_cells:
            raise ValueError("There must be one voltage offset per series cell")
        if len(temperature_offsets_C) != (
            num_modules * thermistors_per_module
        ):
            raise ValueError("There must be one offset per thermistor")

        self.cell_voltage_offsets_mV = cell_voltage_offsets_mV
        self.temperature_offsets_C = temperature_offsets_C

        # cell values used by older code
        self.cell_capacity_ah = cell.capacity_ah
        self.cell_nominal_voltage_v = cell.nominal_voltage_v
        self.cell_internal_resistance_ohm = cell.internal_resistance_ohm
        self.cell_max_discharge_current_a = cell.max_discharge_current_a
        self.ocv_soc = cell.ocv_soc
        self.ocv_cell_voltage_v = cell.ocv_voltage_v

        # values for the whole pack
        self.total_cells = self.getTotalCellCount()
        self.capacity_ah = self.getPackCapacity_Ah()
        self.nominal_voltage_v = self.getPackNominalVoltage_V()
        self.nominal_energy_kwh = self.getPackNominalEnergy_kWh()
        self.internal_resistance_ohm = self.getPackResistance_Ohm()
        self.max_discharge_current_a = self.getRatedPackMaxCurrent_A()
        self.heat_capacity_jpk = self.getPackHeatCapacity_JpK()
        self.min_voltage_v = cell.min_voltage_v * series_cells
        self.max_voltage_v = cell.max_voltage_v * series_cells

        self.reset()

    # gets the number of cells in the pack
    def getTotalCellCount(self):
        return self.series_cells * self.parallel_cells

    # gets pack capacity
    def getPackCapacity_Ah(self):
        return self.parallel_cells * self.cell.capacity_ah

    # gets normal pack voltage
    def getPackNominalVoltage_V(self):
        return self.series_cells * self.cell.nominal_voltage_v

    # gets normal pack energy
    def getPackNominalEnergy_kWh(self):
        watt_hours = self.nominal_voltage_v * self.capacity_ah
        return watt_hours / 1000

    # gets resistance for the whole pack
    def getPackResistance_Ohm(self):
        series_resistance_ohm = (
            self.series_cells * self.cell.internal_resistance_ohm
        )
        return series_resistance_ohm / self.parallel_cells

    # gets the rated max pack current
    def getRatedPackMaxCurrent_A(self):
        return self.parallel_cells * self.cell.max_discharge_current_a

    # gets the energy needed to heat the pack one degree
    def getPackHeatCapacity_JpK(self):
        return self.mass_kg * self.specific_heat_jpkgk

    # resets the pack state
    def reset(self, initial_soc=None, initial_temp_C=None, ambient_temp_C=None):
        if initial_soc is None:
            initial_soc = self.initial_soc
        if initial_temp_C is None:
            initial_temp_C = self.initial_temp_C
        if ambient_temp_C is None:
            ambient_temp_C = self.ambient_temp_C

        if initial_soc < 0 or initial_soc > 1:
            raise ValueError("SOC must be between 0 and 1")

        # 0 is empty and 1 is full
        self.soc = initial_soc

        # pack and outside temperature
        self.temp_C = initial_temp_C
        self.ambient_temp_C = ambient_temp_C

        # current pack state
        self.elapsed_time_s = 0.0
        self.current_A = 0.0
        self.open_circuit_voltage_V = self.getPackOpenCircuitVoltage_V()
        self.terminal_voltage_V = self.open_circuit_voltage_V
        self.terminal_power_W = 0.0
        self.heat_W = 0.0

        # energy counters
        self.discharged_energy_kWh = 0.0
        self.charged_energy_kWh = 0.0
        self.shunt_coulomb_count_C = 0.0

        # saved values for graphs
        self.time_s = [0.0]
        self.soc_history = [self.soc]
        self.temp_history_C = [self.temp_C]
        self.current_history_A = [self.current_A]
        self.voltage_history_V = [self.terminal_voltage_V]
        self.power_history_W = [self.terminal_power_W]

    # runs one pack step
    def update(self, current_A, dt_s):
        if dt_s <= 0:
            raise ValueError("Time step must be positive")

        # positive current drains the pack
        self.open_circuit_voltage_V = self.getPackOpenCircuitVoltage_V()
        self.current_A = current_A
        self.terminal_voltage_V = self.getPackTerminalVoltage_V(current_A)
        self.terminal_power_W = self.getPackPower_W(
            self.terminal_voltage_V,
            current_A,
        )
        self.heat_W = self.getPackHeat_W(current_A)

        next_soc = self.getNextSoc(current_A, dt_s)
        if next_soc < 0 or next_soc > 1:
            raise ValueError("Pack SOC went outside the modeled range")
        self.soc = next_soc

        self.temp_C = self.getTemperatureAfterStep_C(self.heat_W, dt_s)

        # save charge and discharge energy separately
        energy_kWh = self.getEnergyForStep_kWh(self.terminal_power_W, dt_s)
        self.discharged_energy_kWh += max(energy_kWh, 0)
        self.charged_energy_kWh += max(-energy_kWh, 0)

        # update the shunt charge counter
        self.shunt_coulomb_count_C = self.getNextCoulombCount_C(
            self.current_A,
            dt_s,
        )
        self.elapsed_time_s += dt_s

        self.time_s.append(self.elapsed_time_s)
        self.soc_history.append(self.soc)
        self.temp_history_C.append(self.temp_C)
        self.current_history_A.append(self.current_A)
        self.voltage_history_V.append(self.terminal_voltage_V)
        self.power_history_W.append(self.terminal_power_W)

    # gets current through one cell
    def getCellCurrent_A(self, pack_current_A):
        return pack_current_A / self.parallel_cells

    # gets pack voltage with no current flowing
    def getPackOpenCircuitVoltage_V(self, soc=None):
        if soc is None:
            soc = self.soc
        cell_voltage_V = self.cell.getOpenCircuitVoltage_V(soc)
        return cell_voltage_V * self.series_cells

    # gets pack voltage while current is flowing
    def getPackTerminalVoltage_V(self, current_A, soc=None):
        if soc is None:
            soc = self.soc
        cell_current_A = self.getCellCurrent_A(current_A)
        cell_voltage_V = self.cell.getTerminalVoltage_V(soc, cell_current_A)
        return cell_voltage_V * self.series_cells

    # gets pack power
    def getPackPower_W(self, terminal_voltage_V, current_A):
        return terminal_voltage_V * current_A

    # gets heat made by every cell
    def getPackHeat_W(self, pack_current_A):
        cell_current_A = self.getCellCurrent_A(pack_current_A)
        cell_heat_W = self.cell.getHeat_W(cell_current_A)
        return cell_heat_W * self.total_cells

    # gets soc after one step
    def getNextSoc(self, pack_current_A, dt_s):
        cell_current_A = self.getCellCurrent_A(pack_current_A)
        return self.cell.getNextSoc(self.soc, cell_current_A, dt_s)

    # gets energy used during one step
    def getEnergyForStep_kWh(self, power_W, dt_s):
        joules_per_kilowatt_hour = 3_600_000
        return power_W * dt_s / joules_per_kilowatt_hour

    # gets the next shunt charge count
    def getNextCoulombCount_C(self, current_A, dt_s):
        charge_moved_C = current_A * dt_s
        return self.shunt_coulomb_count_C + charge_moved_C

    # gets current needed for a power request
    def currentForPower(self, power_W, voltage_V=None):
        if voltage_V is None:
            voltage_V = self.open_circuit_voltage_V

        # split power between every cell
        cell_power_W = power_W / self.total_cells
        cell_open_circuit_voltage_V = voltage_V / self.series_cells
        cell_current_A = self.cell.getCurrentForPower_A(
            cell_power_W,
            cell_open_circuit_voltage_V,
        )
        return cell_current_A * self.parallel_cells

    # gets cell voltage from soc
    def interpolate(self, soc):
        return self.cell.getOpenCircuitVoltage_V(soc)

    # gets temperature rise with no cooling
    def getAdiabaticTemperatureRise_C(self, heat_W, dt_s):
        heat_energy_J = heat_W * dt_s
        return heat_energy_J / self.heat_capacity_jpk

    # gets the temperature the pack will settle at
    def getSteadyTemperature_C(self, heat_W):
        return self.ambient_temp_C + heat_W * self.thermal_resistance_kpw

    # gets how fast pack temperature changes
    def getThermalTimeConstant_s(self):
        return self.thermal_resistance_kpw * self.heat_capacity_jpk

    # gets pack temperature after one step
    def getTemperatureAfterStep_C(self, heat_W, dt_s):
        if self.thermal_resistance_kpw is None:
            return self.temp_C + self.getAdiabaticTemperatureRise_C(
                heat_W,
                dt_s,
            )

        steady_temperature_C = self.getSteadyTemperature_C(heat_W)
        thermal_time_constant_s = self.getThermalTimeConstant_s()
        remaining_temperature_fraction = math.exp(
            -dt_s / thermal_time_constant_s
        )
        return steady_temperature_C + (
            self.temp_C - steady_temperature_C
        ) * remaining_temperature_fraction

    # old name for the temperature step function
    def temperatureCalc(self, heat_W, dt_s):
        return self.getTemperatureAfterStep_C(heat_W, dt_s)

    # gets pack voltage at the current soc
    def getPackMaxVoltage_V(self):
        return self.getPackOpenCircuitVoltage_V()

    # gets max pack current at the current soc
    def getPackMaxCurrent_A(self):
        cell_max_current_A = self.cell.getMaxDischargeCurrent_A(self.soc)
        return cell_max_current_A * self.parallel_cells

    # bms sensor values
    # gets all cell voltage readings
    def getCellVoltages_mV(self):
        cell_voltage_V = self.terminal_voltage_V / self.series_cells
        voltages_mV = []
        for cell_number in range(self.series_cells):
            voltage_mV = cell_voltage_V * 1000
            voltage_mV += self.cell_voltage_offsets_mV[cell_number]
            voltages_mV.append(round(voltage_mV))
        return voltages_mV

    # gets all temperature readings
    def getTemperatures_C(self):
        temperatures_C = []
        for thermistor_number in range(len(self.temperature_offsets_C)):
            temperature_C = (
                self.temp_C + self.temperature_offsets_C[thermistor_number]
            )
            temperatures_C.append(int(temperature_C))
        return temperatures_C

    # splits sensor readings into modules
    def getModuleData(self):
        cell_voltages_mV = self.getCellVoltages_mV()
        temperatures_C = self.getTemperatures_C()
        modules = []

        for module_number in range(self.num_modules):
            first_cell = module_number * self.cells_per_module
            last_cell = first_cell + self.cells_per_module
            first_thermistor = module_number * self.thermistors_per_module
            last_thermistor = first_thermistor + self.thermistors_per_module
            modules.append({
                "cellVoltage_mV": cell_voltages_mV[first_cell:last_cell],
                "pointTemp_C": temperatures_C[
                    first_thermistor:last_thermistor
                ],
            })
        return modules

    # gets the pack voltage sensor reading
    def getHvSensePackVoltage_cV(self):
        voltage_V = self.terminal_voltage_V + self.hv_sense_offset_V
        return round(voltage_V * 100)

    # gets the current sensor reading
    def getShuntCurrent_mA(self):
        return round(self.current_A * 1000)

    # adds all cell voltage readings
    def getSummedPackVoltage_cV(self, cell_voltages_mV):
        pack_voltage_mV = sum(cell_voltages_mV)
        millivolts_per_centivolt = 10
        return round(pack_voltage_mV / millivolts_per_centivolt)

    # gets everything the bms reads
    def getBMSInputs(self):
        modules = self.getModuleData()
        cell_voltages_mV = []
        for module in modules:
            cell_voltages_mV.extend(module["cellVoltage_mV"])

        return {
            "moduleData": modules,
            "sumPackVoltage_cV": self.getSummedPackVoltage_cV(
                cell_voltages_mV
            ),
            "hvSensePackVoltage_cV": self.getHvSensePackVoltage_cV(),
            "shuntCurrent_mA": self.getShuntCurrent_mA(),
            "shuntCoulombCount_C": self.shunt_coulomb_count_C,
        }
