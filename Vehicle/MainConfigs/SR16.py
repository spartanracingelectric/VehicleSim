from Vehicle.VehicleConfig import VehicleConfig
from Functions.loadConfigs import loadConfigs

batteryConfigs = loadConfigs("Vehicle.Battery.Configs")
axelConfigs = loadConfigs("Vehicle.Axel.Configs")
bmsConfigs = loadConfigs("Vehicle.BMS.Configs")
brakeConfigs = loadConfigs("Vehicle.Brake.Configs")
driverConfigs = loadConfigs("Vehicle.Driver.Configs")
inverterConfigs = loadConfigs("Vehicle.Inverter.Configs")
motorConfigs = loadConfigs("Vehicle.Motor.Configs")
tireConfigs = loadConfigs("Vehicle.Tire.Configs")

SR16 = VehicleConfig(
    vehicle_mass_kg=204.1,
    battery=batteryConfigs.battery1,
    axel=axelConfigs.axel1,
    bms=bmsConfigs.bms1,
    brake=brakeConfigs.brake1,
    driver=driverConfigs.driver1,
    inverter=inverterConfigs.inverter1,
    motor=motorConfigs.motor1,
    tire=tireConfigs.tire1,
)