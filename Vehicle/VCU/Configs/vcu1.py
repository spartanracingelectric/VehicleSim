from Vehicle.VCU.VCUConfig import VCUConfig
from Vehicle.Motor.Configs.amk_dd5 import amk_dd5

vcu1 = VCUConfig(
    loop_period_s=0.01,             # VCU loop runs every 10 ms
    # 100% pedal = the current that makes the AMK's max torque (21 Nm -> 95.6 A with id = 0)
    # TODO: the real firmware's full-pedal current (76.3 A in the SIL tests)
    max_current_request_A=amk_dd5.find_iq_desired(amk_dd5.peakTorque_Nm),
)
