import math

# Frame transforms for three-phase voltages and currents.
# "d/q" alone is the stationary frame; "syn" is the rotor (synchronous) frame.
# these take single numbers (math instead of numpy, numpy is slow on one number at a time)

SQRT3 = math.sqrt(3)

# phase a/b/c -> stationary d/q/0 (amplitude invariant, phase a on the d axis)
def clarke(a, b, c):
    d = (2 * a - b - c) / 3
    q = (b - c) / SQRT3
    zero = (a + b + c) / 3
    return d, q, zero

# stationary d/q/0 -> phase a/b/c
def inverseClarke(d, q, zero=0.0):
    a = d + zero
    b = -0.5 * d + SQRT3 / 2 * q + zero
    c = -0.5 * d - SQRT3 / 2 * q + zero
    return a, b, c

# stationary d/q -> synchronous d/q at electrical angle theta_e_rad
def park(d, q, theta_e_rad):
    cos_theta = math.cos(theta_e_rad)
    sin_theta = math.sin(theta_e_rad)
    d_syn = cos_theta * d + sin_theta * q
    q_syn = -sin_theta * d + cos_theta * q
    return d_syn, q_syn

# synchronous d/q -> stationary d/q at electrical angle theta_e_rad
def inversePark(d_syn, q_syn, theta_e_rad):
    cos_theta = math.cos(theta_e_rad)
    sin_theta = math.sin(theta_e_rad)
    d = cos_theta * d_syn - sin_theta * q_syn
    q = sin_theta * d_syn + cos_theta * q_syn
    return d, q
