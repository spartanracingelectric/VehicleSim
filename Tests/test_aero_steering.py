from Vehicle.Aero.Configs.aero1 import aero1
from Vehicle.Steering.Configs.steering1 import steering1

def test_downforce_at_skidpad_speed():
    # //testing  downforce calculation for 11.588 m/s, car experiences 1.5g of sideways accel on the skidpad circle at this speed, good baseline
    assert abs(aero1.getDownforce_N(11.588) - 389.2) < 0.1

def test_rear_downforce_at_skidpad_speed():
    assert abs(aero1.getRearDownforce_N(11.588) - 199.1) < 0.1

def test_front_downforce_at_skidpad_speed():
    assert abs(aero1.getFrontDownforce_N(11.588) - 190.1) < 0.1


def test_drag_at_skidpad_speed():
    assert abs(aero1.getDrag_N(11.588) - 145.0) < 0.1

WHEELBASE_M = 1.53924
TRACK_M = 1.2065


def test_full_ackermann_left_turn():
    d_FL, d_FR = steering1.getWheelAngles_rad(0.2)
    assert abs(d_FL - 0.2167) < 1e-4
    assert abs(d_FR - 0.1856) < 1e-4


def test_full_ackermann_right_turn():
    d_FL, d_FR = steering1.getWheelAngles_rad(-0.2)
    assert abs(d_FL - (-0.1856)) < 1e-4
    assert abs(d_FR - (-0.2167)) < 1e-4


def test_straight_ahead():
    d_FL, d_FR = steering1.getWheelAngles_rad(0.0)
    assert abs(d_FL) < 1e-4
    assert abs(d_FR) < 1e-4

    
def test_parallel_steer():
    steering1.ackerman_fraction = 0.0
    d_FL, d_FR = steering1.getWheelAngles_rad(0.2)
    assert abs(d_FL - 0.2) < 1e-4
    assert abs(d_FR - 0.2) < 1e-4





