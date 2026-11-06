from Vehicle.Tire.Tire import Tire

# one tire config for both models (dugoff / pacejka), pick with --tire-model or tire1.setModel(...)
# values from nelson's TTC data (Configs/tiretesting2.py on VehSim-1)
# 16" R20, 4 deg static camber, 8 psi, fit at Fz = 667 N, no load sensitivity in mu yet
tire1 = Tire(
    radius_m=0.2032,
    mu=1.5, #TODO: confirm with nelson, his file says 1.2, tejas's SR_18 uses 1.5
    long_stiff_per_load=25.691, #Cx = 25.691 * Fz, TODO: confirm with nelson, his comments have c_long/c_lat swapped
    lat_stiff_per_load=33.959, #Cy = 33.959 * Fz (1/rad)
    # pacejka only. C and E aren't from a fit, they're the usual starting values. TODO: fit C, E to the TTC data
    C_long=1.5, #71% of the peak left when locked/spinning (sin(1.5 * pi / 2))
    E_long=0.0,
    C_lat=1.3, #89% left when sliding
    E_lat=0.0,
    model="dugoff",
)
