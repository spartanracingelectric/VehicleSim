"""Simple cell-level BMS measurements and protection flags for VehicleSim."""

import math
from collections.abc import Sequence


BMS_CELL_OVER_TEMPERATURE_FLAG = 0x04
BMS_CELL_UNDER_VOLTAGE_FLAG = 0x10
BMS_CELL_OVER_VOLTAGE_FLAG = 0x20


class BMSModel:
    """Sample battery measurements and derive the BMS safety status flags."""

    def __init__(self, config):
        self.config = config
        self.readings = None

    def update(self, cell_voltages_V: Sequence[float], cell_temperatures_C: Sequence[float],
               pack_voltage_V: float, pack_current_A: float, soc_percent: float) -> dict:
        """Store one measurement sample and return its summary and safety flags."""
        cell_voltages = tuple(cell_voltages_V)
        cell_temperatures = tuple(cell_temperatures_C)
        if not cell_voltages:
            raise ValueError("at least one cell voltage is required")
        if not cell_temperatures:
            raise ValueError("at least one cell temperature is required")

        measurements = (*cell_voltages, *cell_temperatures, pack_voltage_V, pack_current_A, soc_percent)
        if not all(math.isfinite(value) for value in measurements):
            raise ValueError("BMS measurements must all be finite numbers")

        fault_flags = 0
        if min(cell_voltages) <= self.config.cell_undervoltage_V:
            fault_flags |= BMS_CELL_UNDER_VOLTAGE_FLAG
        if max(cell_voltages) >= self.config.cell_overvoltage_V:
            fault_flags |= BMS_CELL_OVER_VOLTAGE_FLAG
        if max(cell_temperatures) >= self.config.cell_overtemperature_C:
            fault_flags |= BMS_CELL_OVER_TEMPERATURE_FLAG

        self.readings = {
            "pack_voltage_V": pack_voltage_V,
            "pack_current_A": pack_current_A,
            "soc_percent": soc_percent,
            "cell_min_V": min(cell_voltages),
            "cell_max_V": max(cell_voltages),
            "temp_min_C": min(cell_temperatures),
            "temp_max_C": max(cell_temperatures),
            "fault_flags": fault_flags,
        }
        return self.readings.copy()