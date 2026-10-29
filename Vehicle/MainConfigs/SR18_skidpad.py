from Vehicle.SkidPadConfig import SkidPadConfig


SR18_skidpad = SkidPadConfig(
    mass=288.0,                         # kg
    wheelbase=1.53924,                  # m
    front_track=1.2065,                 # m
    rear_track=1.2065,                  # m
    front_lateral_transfer=0.45,        # no unit :p
    yaw_inertia=110.0,                  # kg m^2
    wheel_radius=0.2032,                # m
    wheel_inertia=0.225,                # kg m^2
    rotor_inertia=0.000274,             # kg m^2
    gear_ratio=11.81,                   # no unit :p
    rolling_resistance=0.015,           # no unit :p
    min_slip_speed=1.0,                 # m/s
    tire=None,
    aero=None,
    steering=None,
    limits=None,
)
