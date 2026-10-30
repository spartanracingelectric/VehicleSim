from Vehicle.VCU.RearTVConfig import RearTVConfig

reartv1 = RearTVConfig(
    kp_Nm_per_radps=200.0,     # placeholder, from B4's examples; get real gains from the TV team
    ki_Nm_per_rad=0.0,         # placeholder; tune on the stub car until the TV team answers
    wheelbase_m=1.53924,
    ref_filter_time_s=0.05,
    understeer_gradient=0.0,   # K_us placeholder (neutral steer) until the TV team gives their target understeer gradient
    mode="inside_cut",
)
