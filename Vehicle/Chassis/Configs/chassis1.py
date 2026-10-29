from Vehicle.Chassis.ChassisConfig import ChassisConfig

chassis1 = ChassisConfig(
    drag_area_m2=1.2,                   # TODO: placeholder, Cd * frontal area from aero
    rolling_resistance_coeff=0.015,     # TODO: placeholder
    wheelbase_m=1.53924,                # TODO: from tejas's SR_17 (LocalSim/Accel/SR_17.py on Tejas-01), check for SR18
    cg_height_m=0.312941529,            # TODO: same as above
    rear_weight_fraction=0.509,         # TODO: same as above, 50.9% static weight on the rear
    wheel_inertia_kgm2=0.225,           # TODO: placeholder, half of tejas's 0.45 for both rear wheels + tires + hubs
)
