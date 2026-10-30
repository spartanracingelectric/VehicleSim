
import math

def getTireLoads_N(car, u_mps, ax_mps2, ay_mps2):
    Weight = car.getWeight()
    front_load = Weight * (1 - car.rear_weight_distribution) + car.aero.getFrontDownforce_N(u_mps) - car.mass * ax_mps2 * car.cg_height / car.wheelbase
    rear_load = Weight * car.rear_weight_distribution + car.aero.getRearDownforce_N(u_mps) + car.mass * ax_mps2 * car.cg_height / car.wheelbase

    front_cornering_transfer_between_wheels= car.front_lateral_transfer * car.mass * ay_mps2 * car.cg_height / car.front_track
    rear_cornering_transfer_between_wheels= (1-car.front_lateral_transfer)* car.mass * ay_mps2 * car.cg_height / car.rear_track

    FL = max(0.0,front_load/2-front_cornering_transfer_between_wheels)
    FR = max(0.0, front_load/2+front_cornering_transfer_between_wheels)
    RL = max(0.0, rear_load/2-rear_cornering_transfer_between_wheels)
    RR = max(0.0, rear_load/2+rear_cornering_transfer_between_wheels)


    return FL, FR, RL, RR

def getCornerSlips(car, forward_speed_mps, sideways_slide_speed_mps, yaw_rate_radps, wheel_speeds_radps, steer_angles_rad):
    slip_angles= []
    slip_ratios= []
    wheel_forward_speeds= []
    
    for i in range(4):
        x_i, y_i = car.getWheelPositions(i)
        d_i = steer_angles_rad[i]
        vx_corner= forward_speed_mps - yaw_rate_radps * y_i
        vy_corner= sideways_slide_speed_mps + yaw_rate_radps * x_i

        vx_w =  vx_corner * math.cos(d_i) + vy_corner * math.sin(d_i)
        vy_w = -vx_corner * math.sin(d_i) + vy_corner * math.cos(d_i)
        
        slip_angle = -math.atan2(vy_w, vx_w)
        slip_ratio = (wheel_speeds_radps[i] * car.wheel_radius - vx_w) / max(abs(vx_w), car.min_slip_speed)


        slip_angles.append(slip_angle)
        slip_ratios.append(slip_ratio)
        wheel_forward_speeds.append(vx_w)


    return slip_ratios, slip_angles, wheel_forward_speeds


