from Vehicle.SkidPadConfig import SkidPadConfig

from Vehicle.Tire.Configs.tire_r20 import tire_r20
from Vehicle.Aero.Configs.aero1 import aero1
from Vehicle.Steering.Configs.steering1 import steering1

SR18_skidpad = SkidPadConfig(
    mass=277.6,                         # kg
    wheelbase=1.530,                    # m
    rear_weight_distribution=0.477,     # unitless
    cg_height=0.303,                    # m
    front_track=1.219,                  # m
    rear_track=1.194,                   # m
    front_lateral_transfer=0.413,       # no unit :p
    yaw_inertia=115.0,                  # kg m^2 (verify)
    wheel_radius=0.2032,                # m
    wheel_inertia=0.225,                # kg m^2
    rotor_inertia=0.000274,             # kg m^2
    gear_ratio=11.81,                   # no unit :p
    rolling_resistance=0.015,           # no unit :p
    min_slip_speed=1.0,                 # m/s
    tire=tire_r20,
    aero=aero1,
    steering=steering1,
    limits=None,
)
