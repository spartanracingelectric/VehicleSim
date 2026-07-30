class BatteryConfig:
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

        # I acutally need to find the curve somewhere tho
        self.ocv_soc = [0.00, 0.05, 0.10, 0.20, 0.50, 0.80, 0.90, 1.00]
        self.ocv_cell_voltage_v = [
            2.50,
            3.20,
            3.40,
            3.55,
            3.70,
            3.88,
            4.00,
            4.20,
        ]
        self.min_voltage_v = series_cells * self.ocv_cell_voltage_v[0]
        self.max_voltage_v = series_cells * self.ocv_cell_voltage_v[-1]
