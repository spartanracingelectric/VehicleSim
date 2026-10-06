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
vcuConfigs = loadConfigs("Vehicle.VCU.Configs")

SR17 = VehicleConfig(
    vehicle_mass_kg=204.1,
    battery=batteryConfigs.sr17,
    axel=axelConfigs.axel1,
    bms=bmsConfigs.bms1,
    brake=brakeConfigs.brake1,
    driver=driverConfigs.driver1,
    inverter=inverterConfigs.inverter1,
    motor=motorConfigs.amk_dd5,
    num_motors=4,    # 4 AMK hub motors, each with its own inverter
    gear_ratio=11.81,    # motor to wheel, per hub motor
    tire=tireConfigs.tire1,
    vcu=vcuConfigs.vcu1,
    drag_area_m2=1.2,    # TODO: placeholder, Cd * frontal area from aero
    rolling_resistance_coeff=0.015,    # TODO: placeholder
)
