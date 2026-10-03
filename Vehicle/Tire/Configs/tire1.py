from Vehicle.Tire.TireConfig import TireConfig

# dugoff tire config, values from nelson's TTC data (Configs/tiretesting2.py on VehSim-1)
# 16" R20, 4 deg static camber, 8 psi, fit at Fz = 667 N, no load sensitivity in mu yet
tire1 = TireConfig(
    radius_m=0.2032,
    longitudinal_stiffness_per_load=25.691, #per load, Cx = 25.691 * Fz, TODO: confirm with nelson, his comments have c_long/c_lat swapped
    lateral_stiffness_per_load=33.959, #per load, Cy = 33.959 * Fz (1/rad)
    tireroadfriction=1.5, #TODO: confirm with nelson, his file says 1.2, tejas's SR_18 uses 1.5
)
