from Vehicle.Tire.TireConfig import TireConfig

# dugoff tire config, stiffness and friction values need to come from real tire data
tire1 = TireConfig(
    radius_m=0.4064, #placeholder
    longitudinal_stiffness=50000.0, #placeholder, N per unit slip
    lateral_stiffness=40000.0, #placeholder, N/rad
    tireroadfriction=1.5, #placeholder
)
