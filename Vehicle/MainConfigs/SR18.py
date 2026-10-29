from copy import deepcopy
from Vehicle.VehicleConfig import VehicleConfig
from Functions.loadConfigs import loadConfigs

batteryConfigs = loadConfigs("Vehicle.Battery.Configs")
axelConfigs = loadConfigs("Vehicle.Axel.Configs")
bmsConfigs = loadConfigs("Vehicle.BMS.Configs")
brakeConfigs = loadConfigs("Vehicle.Brake.Configs")
chassisConfigs = loadConfigs("Vehicle.Chassis.Configs")
driverConfigs = loadConfigs("Vehicle.Driver.Configs")
inverterConfigs = loadConfigs("Vehicle.Inverter.Configs")
motorConfigs = loadConfigs("Vehicle.Motor.Configs")
tireConfigs = loadConfigs("Vehicle.Tire.Configs")
vcuConfigs = loadConfigs("Vehicle.VCU.Configs")

SR18 = VehicleConfig(
    vehicle_mass_kg=204.1,
    battery=batteryConfigs.sr17,
    axel=axelConfigs.axel1,
    bms=bmsConfigs.bms1,
    brake=brakeConfigs.brake1,
    chassis=chassisConfigs.chassis1,
    driver=driverConfigs.driver1,
    # 4 AMK hub motors, each with its own inverter: front left, front right, rear left, rear right
    # copies, since each one keeps its own currents, angle and controller memory
    inverters=[deepcopy(inverterConfigs.inverter1) for _ in range(4)],
    motors=[deepcopy(motorConfigs.amk_dd5) for _ in range(4)],
    num_motors=4,
    gear_ratio=11.81,    # motor to wheel, per hub motor
    tire=tireConfigs.tire1,
    vcu=vcuConfigs.vcu1,
)
