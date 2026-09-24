from Vehicle.Motor.MotorConfig import MotorConfig

# old constants
emrax228_mv = MotorConfig(
    Kt=0.64,
    peakCurrent_A=360,
    peakTorque_Nm=220,
    peak_power_W=124000,
    max_rpm=6500
)

# new constants
amk_dd5 = MotorConfig(
    Kt = 0.26,
    peakCurrent_A = 105,
    peakTorque_Nm = 21,
    peak_power_W = 12300, #rated power, not true peak power
    max_rpm = 20000
) 