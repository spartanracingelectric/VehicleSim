from Vehicle.Aero.Configs.aero1 import aero1

def test_downforce_at_skidpad_speed():
    # //testing  downforce calculation for 11.588 m/s, car experiences 1.5g of sideways accel on the skidpad circle at this speed, good baseline
    assert abs(aero1.getDownforce_N(11.588) - 389.2) < 0.1

def test_rear_downforce_at_skidpad_speed():
    assert abs(aero1.getRearDownforce_N(11.588) - 199.1) < 0.1

def test_front_downforce_at_skidpad_speed():
    assert abs(aero1.getFrontDownforce_N(11.588) - 190.1) < 0.1


def test_drag_at_skidpad_speed():
    assert abs(aero1.getDrag_N(11.588) - 145.0) < 0.1
