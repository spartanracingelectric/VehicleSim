from Vehicle.Tire.TireConfig2 import TireConfig

#Assumptions:
#4 deg static camber, 8psi tires parsed from TTC 667N
#NO TIRE LOAD SENSITIVITY
#16" longitudinal tire behavior extrapolated from 20" & 18" R20 behavior

tire1 = TireConfig(
    radius_m = 0.2032,
    c_long = 25.691, #cornering stiffness N/rad divided by normal load
    c_lat = 33.959, #longitudinal stiffnes N/rad divided by normal load
    tiremu = 1.2
)
