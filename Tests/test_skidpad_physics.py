from Vehicle.MainConfigs.SR18_skidpad import SR18_skidpad
from Functions.rk4 import rk4Step
from Vehicle.Chassis.PlanarChassis import getTireLoads_N
from Vehicle.Driver.SkidPadDriverConfig import SkidPadDriverConfig

def runCircle(speed, turn="left", dt_s=0.0005, time_s=5.0):
    radius = 9.125
    x, center = SR18_skidpad.getStartStateOnCircle(speed, radius, turn)

    driver = SkidPadDriverConfig(mode="circle", target_speed_mps=lambda t: speed,
                                 center_m=center, radius_m=radius, turn=turn,
                                 lookahead_m=3.0, wheelbase_m=SR18_skidpad.wheelbase,
                                 speed_kp=30.0, speed_ki=10.0, max_wheeltorque_Nm=248.0)
    
    inputs = {"steer_angles_rad": [0.0, 0.0, 0.0, 0.0], "wheel_torques_Nm": [0.0, 0.0, 0.0, 0.0],
              "external_yaw_moment_Nm": 0.0, "ax_mps2": 0.0, "ay_mps2": 0.0}
    steps_per_update = round(0.01 / dt_s)

    for step in range(round(time_s / dt_s)):
        if step % steps_per_update == 0:
            delta, torque = driver.update(x, step * dt_s, 0.01)
            d_FL, d_FR = SR18_skidpad.steering.getWheelAngles_rad(delta)
            inputs["steer_angles_rad"] = [d_FL, d_FR, 0.0, 0.0]
            inputs["wheel_torques_Nm"] = [0.0, 0.0, torque, torque]
        x, outputs = rk4Step(x, inputs, SR18_skidpad, dt_s)
        inputs["ax_mps2"] = outputs["ax_mps2"]
        inputs["ay_mps2"] = outputs["ay_mps2"]

    return x, outputs, delta


def test_coast_down():
    wheel_speed = 10.0 / SR18_skidpad.wheel_radius
    x = [0.0, 0.0, 0.0, 10.0, 0.0, 0.0, wheel_speed, wheel_speed, wheel_speed, wheel_speed]
    inputs = {
    "steer_angles_rad": [0.0, 0.0, 0.0, 0.0], 
    "wheel_torques_Nm": [0.0, 0.0, 0.0, 0.0],  
    "external_yaw_moment_Nm": 0.0,                
    "ax_mps2": 0,                               
    "ay_mps2": 0.0,
}
    for step in range(2000):
        x,outputs= rk4Step(x, inputs, SR18_skidpad, .0005)
        inputs["ax_mps2"] = outputs["ax_mps2"]
        inputs["ay_mps2"] = outputs["ay_mps2"]
    assert abs(x[3] - 9.5125) < 0.005
    for slip in outputs["slip_ratios"]:
        assert abs(slip) < 0.001

def test_loads():
    totalload=sum(getTireLoads_N(SR18_skidpad, 11.0, 0.0, 9.81))
    
    assert abs(totalload - SR18_skidpad.getWeight()-SR18_skidpad.aero.getDownforce_N(11.0)) < 1

def test_lateral_transfer():
    loads = getTireLoads_N(SR18_skidpad, 11.0, 0.0, 9.81)
    front_transfer =SR18_skidpad.front_lateral_transfer * SR18_skidpad.mass * 9.81 * SR18_skidpad.cg_height/SR18_skidpad.front_track
    rear_transfer = (1-SR18_skidpad.front_lateral_transfer) * SR18_skidpad.mass * 9.81 * SR18_skidpad.cg_height/SR18_skidpad.rear_track
    assert abs((loads[1]-loads[0])/2 - front_transfer)< .02 *front_transfer
    assert abs((loads[3]-loads[2])/2 - rear_transfer)< .02 *rear_transfer


def test_force_balance():
    x,outputs,delta= runCircle(10.0) 
    assert abs(outputs["ay_mps2"]-(x[3]*x[5])) < .01

def test_steering():
     x,outputs,delta= runCircle(5.18)
     assert abs(delta-SR18_skidpad.wheelbase/9.125) < delta*.1

def test_left_right():
    x,outputs,delta=runCircle(10.0, "left")
    x2,outputs2,delta2=runCircle(10.0, "right")

    assert abs(x[3] - x2[3]) < .00001 and abs(x[5] + x2[5]) < .00001
    assert abs(delta + delta2) < .00001 and abs(outputs["ay_mps2"] + outputs2["ay_mps2"]) < .00001

def test_time_step():
    x,outputs,delta=runCircle(10.0)
    x2,outputs2,delta2=runCircle(10.0, dt_s=.00025)

    assert abs(x[3]-x2[3])<.00001*x[3] and abs(x[5]-x2[5])<.00001*x[5] and abs(outputs["ay_mps2"]-outputs2["ay_mps2"])<.00001*outputs["ay_mps2"]