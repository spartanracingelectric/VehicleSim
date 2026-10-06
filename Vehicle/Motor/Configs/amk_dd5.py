from Vehicle.Motor.MotorConfig import MotorConfig

# AMK DD5-14-10-POW (Formula Student), values from amk_motor_datasheet.pdf in this folder
amk_dd5 = MotorConfig(
    # Kt = 0.26,
    # stator_resistance_ohm = 0.135,
    # num_poles = 10, #poles
    # quadrature_axis_inductance_H = 0.00012, #millihenries (DS) -> Henries
    # direct_axis_inductance_H = 0.00024,
    # rotor_intertia_kgm2 = 0.000274 #kgcm^2 (DS) ->kgm^2

    Kt = 0.26, #torque constant (Nm/Arms), datasheet, just for reference (the model uses lambda_f)
    Rs = 0.0675, #per phase (ohm): the datasheet gives terminal-to-terminal Rtt = 0.135, per phase is half
    Np = 10, #number of poles
    # the datasheet lists Lq 0.12 / Ld 0.24 mH, but its torque-speed curves only fit the other way round
    # (Lq 0.24: ~1 Nm rms off the curves, Lq 0.12: 4.4 Nm off), and Lq > Ld is normal for this kind of motor
    Lq = 0.00024, #quadrature axis inductance (H)
    Ld = 0.00012, #direct axis inductance (H)
    Jm = 0.000274, #inertia (kgm^2)
    # from ke = 18.8 V/1000 rpm (line rms): gives the datasheet's 19000 rpm zero-torque speed at 500 V
    # (Kt would give 0.0245, but that back-emf is 16% too low)
    lambda_f = 0.0293, #rotor flux linkage (Wb)
    peakTorque_Nm = 21.0, #max torque per motor, before the gearbox
    maxCurrent_Arms = 105.0, #for up to 1.24 s, continuous stall current is 53.1
    maxSpeed_rpm = 20000.0, #mechanical speed limit
    fieldWeakeningCurrent_Arms = 35.0, #"magn. current Im", how far id can go negative for field weakening
)
