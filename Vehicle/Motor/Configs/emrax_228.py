from Vehicle.Motor.MotorConfig import MotorConfig

# EMRAX 228 MV (the motor on SR17 in the Crows log)
# TODO: MotorConfig now needs the electrical values, fill these in from the EMRAX datasheet:
#   Kt, Rs (ohm), Np (number of poles), Lq (H), Ld (H), Jm (kgm^2), lambda_f (Wb), peakTorque_Nm (220 from the old values)
# old values, from the torque-only model:
#   Kt=0.64, peakCurrent_A=360, peakTorque_Nm=220, peak_power_W=124000, max_rpm=6500
#
# emrax_228 = MotorConfig(
#     Kt=,
#     Rs=,
#     Np=,
#     Lq=,
#     Ld=,
#     Jm=,
#     lambda_f=,
#     peakTorque_Nm=220,
# )

emrax_228 = None    # placeholder until the values above are filled in
