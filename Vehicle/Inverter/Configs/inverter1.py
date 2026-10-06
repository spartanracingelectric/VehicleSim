from Vehicle.Inverter.InverterConfig import InverterConfig


inverter1 = InverterConfig(
    con_current_A=300.0,
    max_dc_voltage_V=900.0,
    efficiency=0.97,    # TODO: placeholder, replace with the real inverter's number
    modulation="svm",
    switching_frequency_Hz=10000.0,    # TODO: placeholder, replace with the real inverter's number

    # current controller (moved from amk_dd5.py)
    current_kp_d=0.58,      # d-axis proportional gain (V/A)
    current_kp_q=0.64,      # q-axis proportional gain (V/A)
    current_tn_d_ms=1.2,    # d-axis integral time (ms)
    current_tn_q_ms=1.2,    # q-axis integral time (ms)
)
