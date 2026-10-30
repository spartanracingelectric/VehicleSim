from Vehicle.VCU.RearTVConfig import RearTVConfig

rear_tv1 = RearTVConfig(
    kp_Nm_per_radps=200.0,     # placeholder, from B4's examples; get real gains from the TV team
    ki_Nm_per_rad=0.0,         # placeholder; tune on the stub car until the TV team answers
    wheelbase_m=1.530,
    ref_filter_time_s=0.05,
    # K_us in rad per m/s^2: -0.25 deg/g (slight oversteer) up to 10 m/s, ramps to +0.25 deg/g (slight understeer) at 20 m/s (verify)
    understeer_gradient=lambda u_mps: -0.000445 + 0.000890 * min(max((u_mps - 10.0) / 10.0, 0.0), 1.0),
    mode="inside_cut",
    torque_rate_limit_Nm_per_s=5000.0,   # Nm/s at the wheel, 248 Nm to 0 in ~50 ms (verify)
)
