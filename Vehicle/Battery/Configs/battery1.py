from Vehicle.Battery.BatteryConfig import BatteryConfig

battery1 = BatteryConfig(
    mass_kg=50.8,
    series_cells=140,
    parallel_cells=3,
    cell_capacity_ah=5.0,
    cell_nominal_voltage_v=3.6,
    cell_internal_resistance_ohm=6.7e-3,
    cell_max_discharge_current_a=125.0,
)
