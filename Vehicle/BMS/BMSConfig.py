from Vehicle.Battery.Battery import Battery

class BMSConfig:
    def __init__(self, max_cell_voltage_threshold_mV : float, min_cell_voltage_threshold_mV : float, max_temp_threshold_C : float, min_temp_threshold_C : float, max_power_threshold_kW : float):
        self.max_cell_voltage_threshold_mV = max_cell_voltage_threshold_mV;
        self.min_cell_voltage_threshold_mV = min_cell_voltage_threshold_mV;
        self.max_temp_threshold_C = max_temp_threshold_C;
        self.min_temp_threshold_C = min_temp_threshold_C;
        self.max_power_threshold_kW = max_power_threshold_kW

        self.cell_voltages_mV = []
        self.temperatures_C = []

        self.power_kW = 0.0
        self.current_mA = 0.0

        self.max_voltage_mV = 0.0
        self.min_voltage_mV = 0.0
        self.average_voltage_mV = 0.0
        self.total_voltage_mV = 0.0

        self.max_temperature_C = 0.0
        self.min_temperature_C = 0.0
        self.average_temperature_C = 0.0

    def update(self, battery: Battery) -> None:
        self.update_voltages(battery.getCellVoltages_mV())
        self.update_temperatures(battery.getTemperatures_C())
        self.update_current(battery.getShuntCurrent_mA())
    
    def update_voltages(self, voltages_mV : list) -> None:
        self.cell_voltages_mV = voltages_mV.copy()
        self.update_voltage_metrics()
        self.update_power()

    def update_temperatures(self, temperatures_C : list) -> None:
        self.temperatures_C = temperatures_C.copy()
        self.update_temperature_metrics()

    def update_voltage_metrics(self) -> None:
        if not self.cell_voltages_mV:
            return

        self.max_voltage_mV = max(self.cell_voltages_mV)
        self.min_voltage_mV = min(self.cell_voltages_mV)
        self.total_voltage_mV = sum(self.cell_voltages_mV)
        self.average_voltage_mV = (self.total_voltage_mV / len(self.cell_voltages_mV))
    
    def update_temperature_metrics(self) -> None:
        if not self.temperatures_C:
            return

        self.max_temperature_C = max(self.temperatures_C)
        self.min_temperature_C = min(self.temperatures_C)
        self.average_temperature_C = (sum(self.temperatures_C) / len(self.temperatures_C))

    def update_current(self, current_mA: float) -> None:
        self.current_mA = current_mA
        self.update_power()

    def update_power(self) -> None:
        self.power_kW = (self.total_voltage_mV * self.current_mA) / 1000000000.0

    def has_overvoltage_fault(self) -> bool:
        if not self.cell_voltages_mV:
            return False

        return (self.max_voltage_mV > self.max_cell_voltage_threshold_mV)

    def has_undervoltage_fault(self) -> bool:
        if not self.cell_voltages_mV:
            return False

        return (self.min_voltage_mV < self.min_cell_voltage_threshold_mV)

    def has_overtemperature_fault(self) -> bool:
        if not self.temperatures_C:
            return False

        return (self.max_temperature_C > self.max_temp_threshold_C)

    def has_undertemperature_fault(self) -> bool:
        if not self.temperatures_C:
            return False

        return (self.min_temperature_C < self.min_temp_threshold_C)

    def has_overpower_fault(self) -> bool:
        return self.power_kW > self.max_power_threshold_kW

    def getCurrent_mA(self) -> float:
        return self.current_mA
    
    def getVoltages_mV(self) -> list[float]:
        return self.cell_voltages_mV

    def getTemperatures_C(self) -> list[float]:
        return self.temperatures_C

