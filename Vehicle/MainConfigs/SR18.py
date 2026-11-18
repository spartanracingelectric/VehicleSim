"""Supported longitudinal electrical car. Every factory call owns its state."""
from copy import deepcopy
from Vehicle.VehicleConfig import VehicleConfig
from Vehicle.Battery.Configs.sr17 import create_sr17_battery
from Vehicle.Axel.Configs.axel1 import axel1
from Vehicle.BMS.Configs.bms1 import bms1
from Vehicle.Brake.Configs.brake1 import brake1
from Vehicle.Chassis.Configs.chassis1 import chassis1
from Vehicle.Driver.Configs.driver1 import driver1
from Vehicle.Inverter.Configs.vesc_srinverter import vesc_srinverter
from Vehicle.Motor.Configs.amk_dd5 import amk_dd5
from Vehicle.Tire.Configs.tire1 import tire1
from Vehicle.VCU.Configs.vcu1 import vcu1


def create_sr18(*, battery=None, battery_thermal_model=None, initial_temp_C=25.0, ambient_temp_C=25.0):
    """Create a fresh four-drive electrical car with the longitudinal chassis."""
    if battery is None:
        selected_model = "lptn" if battery_thermal_model is None else battery_thermal_model
        battery = create_sr17_battery(thermal_model=selected_model)
    else:
        if battery_thermal_model is not None:
            raise ValueError("Choose the thermal model when creating a custom battery")
        battery = deepcopy(battery)
    battery.reset(initial_temp_C=initial_temp_C, ambient_temp_C=ambient_temp_C)
    return VehicleConfig(
        vehicle_mass_kg=204.1, gear_ratio=11.81, num_motors=4,
        battery=battery, axel=deepcopy(axel1), bms=deepcopy(bms1),
        brake=deepcopy(brake1), chassis=deepcopy(chassis1), driver=deepcopy(driver1),
        inverters=[deepcopy(vesc_srinverter) for _ in range(4)],
        motors=[deepcopy(amk_dd5) for _ in range(4)],
        tire=deepcopy(tire1), vcu=deepcopy(vcu1),
    )


# Compatibility object for existing consumers; new code should use the factory.
SR18 = create_sr18()
