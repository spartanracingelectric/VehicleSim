from Vehicle.BMS.BMSConfig import BMSConfig

bms1 = BMSConfig(
    max_cell_voltage_threshold_mV = 4200.0,
    min_cell_voltage_threshold_mV = 2800.0,
    max_temp_threshold_C = 60.0,
    min_temp_threshold_C = 0.0,
    max_power_threshold_kW = 80.0
)

bms1.update_voltages([3950.0, 4000.0, 4210.0, 3900.0])
bms1.update_temperatures([30.0, 35.0, 61.0, 32.0])
bms1.update_current(100000.0)

print("Voltage metrics:")
print("Maximum cell voltage:", bms1.max_voltage_mV, "mV")
print("Minimum cell voltage:", bms1.min_voltage_mV, "mV")
print("Average cell voltage:", bms1.average_voltage_mV, "mV")
print("Total pack voltage:", bms1.total_voltage_mV, "mV")

print("\nTemperature metrics:")
print("Maximum temperature:", bms1.max_temperature_C, "C")
print("Minimum temperature:", bms1.min_temperature_C, "C")
print("Average temperature:", bms1.average_temperature_C, "C")

print("\nPower metrics:")
print("Current:", bms1.current_mA, "mA")
print("Power:", bms1.power_kW, "kW")

print("\nFaults:")
print("Overvoltage:", bms1.has_overvoltage_fault())
print("Undervoltage:", bms1.has_undervoltage_fault())
print("Overtemperature:", bms1.has_overtemperature_fault())
print("Undertemperature:", bms1.has_undertemperature_fault())
print("Overpower:", bms1.has_overpower_fault())
