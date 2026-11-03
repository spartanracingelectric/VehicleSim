from Vehicle.Inverter.InverterConfig import InverterConfig

# SR custom inverter, running VESC firmware (SR-Custom-Inverter-FW), one per AMK motor.
# anything left out is the firmware's compiled-in default (see InverterConfig): 26 kHz f_zv sampled in v0 and v7
# (13 kHz switching, 26 kHz current loop), 590 V max, 180 A abs fault, max duty 0.95, 100k erpm (= 20 000 rpm
# with 5 pole pairs) fading from 80 %, 60 A battery current per inverter (hw_SRINVERTER.h).
# what's below is what VESC Tool would need to be set to for the AMK, the firmware defaults are in the comments.
# TODO: swap in what actually gets flashed
vesc_srinverter = InverterConfig(
    efficiency=0.97,    # TODO: placeholder, replace with the real inverter's number

    # default 60 A: the AMK takes 105 A rms = 148.5 A peak (for up to 1.24 s)
    current_max_A=148.5,
    current_min_A=-148.5,

    # default Kp 0.03, Ki 50. motor detection sets Kp = L * bw and Ki = R * bw (default time constant 1000 us),
    # with L = (Ld + Lq) / 2 = 0.18 mH and R = 0.0675 ohm for the AMK
    current_kp=0.18,
    current_ki=67.5,

    # default off. without it the integrator has to build up the back-emf itself and the cross terms
    # (we * L = 0.75 ohm at 6000 rpm) are 4x Kp, so iq rings far past the request (Tests/vescInverterTest.py)
    decoupling="cross_bemf",

    # default 0 A = no field weakening. the AMK allows 35 A rms "magn. current Im" = 49.5 A peak
    fw_current_max_A=49.5,
)
