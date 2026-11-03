"""Supported longitudinal electrical car. Every factory call owns its state."""
from copy import deepcopy
from Vehicle.VehicleConfig import VehicleConfig
from Vehicle.Battery.Configs.sr17 import sr17
from Vehicle.Axel.Configs.axel1 import axel1
from Vehicle.BMS.Configs.bms1 import bms1
from Vehicle.Brake.Configs.brake1 import brake1
from Vehicle.Chassis.Configs.chassis1 import chassis1
from Vehicle.Driver.Configs.driver1 import driver1
from Vehicle.Inverter.Configs.vesc_srinverter import vesc_srinverter
from Vehicle.Motor.Configs.amk_dd5 import amk_dd5
from Vehicle.Tire.Configs.tire1 import tire1
from Vehicle.VCU.Configs.vcu1 import vcu1


def create_sr18():
    """Create a fresh four-drive electrical car with the longitudinal chassis."""
    return VehicleConfig(
        vehicle_mass_kg=204.1, gear_ratio=11.81, num_motors=4,
        battery=deepcopy(sr17), axel=deepcopy(axel1), bms=deepcopy(bms1),
        brake=deepcopy(brake1), chassis=deepcopy(chassis1), driver=deepcopy(driver1),
        inverters=[deepcopy(vesc_srinverter) for _ in range(4)],
        motors=[deepcopy(amk_dd5) for _ in range(4)],
        tire=deepcopy(tire1), vcu=deepcopy(vcu1),
    )


# Compatibility object for existing consumers; new code should use the factory.
SR18 = create_sr18()
