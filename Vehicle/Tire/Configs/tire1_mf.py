from Vehicle.Tire.MagicFormulaTireConfig import MagicFormulaTireConfig

# magic formula version of tire1 (dugoff), same TTC numbers so the two agree at small slip and at the peak
# 16" R20, 4 deg static camber, 8 psi, fit at Fz = 667 N, no load sensitivity in mu yet
# C and E aren't from a fit, they're the usual starting values. TODO: fit C, E to the TTC data
tire1_mf = MagicFormulaTireConfig(
    radius_m=0.2032,
    mu=1.5, #same as tire1, TODO: confirm with nelson (his file says 1.2)
    C_long=1.5, #71% of the peak left when locked/spinning (sin(1.5 * pi / 2))
    E_long=0.0,
    long_stiff_per_load=25.691, #same as tire1, Cx = 25.691 * Fz
    C_lat=1.3, #usual starting shape for lateral, 89% left when sliding
    E_lat=0.0,
    lat_stiff_per_load=33.959, #same as tire1, Cy = 33.959 * Fz (1/rad)
)
