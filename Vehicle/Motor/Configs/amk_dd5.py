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
    # winding temperature, one node. datasheet duty "S1 dT=100K": at the continuous stall current (53.1 Arms)
    # the windings settle 100 K over the coolant. copper loss there = 3 * Rs * I^2 = 3 * 0.0675 * 53.1^2 = 571 W
    # (at stall, so no core loss in that rating), so Rth = 100 / 571.
    # Rs is fixed in this model: if Rs ever rises with temperature, this has to drop to ~0.126
    # (the same 53.1 A makes ~794 W once the copper is at 125 C)
    thermalResistance_KperW = 0.175, #windings -> coolant (K/W)
    # PLACEHOLDER, not on the datasheet: a guess from the copper and stator iron in a 3.55 kg motor
    # (somewhere in 200-800 J/K), gives a 70 s time constant. fit it from a dyno heat-up or a log
    # with the inverter's motor temperature (KTY84): time to 63% of the rise = Rth * Cth
    heatCapacity_JperK = 400.0, #(J/K)
    coolantTemp_C = 25.0, #coolant going into the motor (C)
)