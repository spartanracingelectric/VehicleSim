from Vehicle.Motor.MotorConfig import MotorConfig

# old constants
# emrax228_mv = MotorConfig(
#     Kt=0.64,
#     peakCurrent_A=360,
#     peakTorque_Nm=220,
#     peak_power_W=124000,
#     max_rpm=6500
# )

# new constants
amk_dd5 = MotorConfig(
    # Kt = 0.26,
    # stator_resistance_ohm = 0.135,
    # num_poles = 10, #poles
    # quadrature_axis_inductance_H = 0.00012, #millihenries (DS) -> Henries
    # direct_axis_inductance_H = 0.00024, 
    # rotor_intertia_kgm2 = 0.000274 #kgcm^2 (DS) ->kgm^2

    Kt = 0.26,
    Rs = 0.135, #stator winding resistance
    Np = 10, #number of poles
    Lq = 0.00012, #quadrature axis inductance (H)
    Ld = 0.00024, #direct axis inductance (H)
    Jm = 0.000274 #inertia (kgm^2)
) 