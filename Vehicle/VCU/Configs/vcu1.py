from Vehicle.VCU.VCUConfig import VCUConfig

vcu1 = VCUConfig(
    loop_period_s=0.01,             # VCU loop runs every 10 ms
    max_torque_request_Nm=21.0,     # 100% pedal = the AMK motor's max torque
)
