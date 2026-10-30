from Vehicle.Motor.TorqueLimitConfig import TorqueLimitConfig

# AMK DD5-14-10-POW curve, per motor (from Tejas-01: LocalSim/Resources/Car_Setups/SR_18.py)

amk_limits = TorqueLimitConfig(
    rpm_points=[0, 12000, 15000, 18617],
    torque_points_Nm=[21.0, 21.0, 16.8, 0.0],
    gear_ratio=11.81,
    power_limit_W=80000.0,      # FSAE accumulator power limit
    efficiency=1.0,             # placeholder, no drivetrain losses yet
)
