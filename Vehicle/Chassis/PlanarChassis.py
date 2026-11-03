
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


def derivatives(x, inputs, car):
    loads = getTireLoads_N(car, x[3], inputs["ax_mps2"], inputs["ay_mps2"])
    slip_ratios, slip_angles, wheel_forward_speeds = getCornerSlips(car, x[3], x[4], x[5], x[6:10], inputs["steer_angles_rad"])
    Force_x_tire = []
    Force_y_tire = []
    Force_x_body = []
    Force_y_body = []
    Tire_pos_x = []
    Tire_pos_y = []
    for i in range(4):
        x_i, y_i = car.getWheelPositions(i)
        Tire_pos_x.append(x_i)
        Tire_pos_y.append(y_i)
    for i in range(4):
        Force_x_tire_i, Force_y_tire_i = car.tire.calculateForces(loads [i], slip_ratios[i], slip_angles[i])
        Force_x_tire.append(Force_x_tire_i)
        Force_y_tire.append(Force_y_tire_i)
        Force_x_body_i= Force_x_tire_i * math.cos(inputs["steer_angles_rad"][i]) - Force_y_tire_i * math.sin(inputs["steer_angles_rad"][i])
        Force_y_body_i= Force_x_tire_i * math.sin(inputs["steer_angles_rad"][i]) + Force_y_tire_i * math.cos(inputs["steer_angles_rad"][i])
        Force_x_body.append(Force_x_body_i)
        Force_y_body.append(Force_y_body_i)
    Sum_Fx_body = sum(Force_x_body)
    Sum_Fy_body = sum(Force_y_body)
    Sum_Mz_body = sum(Force_y_body[i] * Tire_pos_x[i] - Force_x_body[i] * Tire_pos_y[i] for i in range(4)) + inputs["external_yaw_moment_Nm"]

    drag= car.aero.getDrag_N(x[3])
    roll=car.rolling_resistance * sum(loads) if x[3] > 0 else 0.0

    forward_velocity_ROF= (Sum_Fx_body - drag - roll) / car.mass + x[4]*x[5]
    lateral_velocity_ROF= (Sum_Fy_body) / car.mass - x[3]*x[5]
    yaw_rate_ROF= Sum_Mz_body / car.yaw_inertia

    x_rof= x[3]*math.cos(x[2]) - x[4]*math.sin(x[2])
    y_rof= x[3]*math.sin(x[2]) + x[4]*math.cos(x[2])

    yaw_rate= x[5]

    wheel_angular_accel= []
    for i in range(4):
        T_i = inputs["wheel_torques_Nm"][i]
        wheel_angular_accel.append((T_i - Force_x_tire[i] * car.wheel_radius) / car.getWheelSpinInertia())

    outputs = {
        "Fz_N": loads,
        "Fx_N": Force_x_tire,
        "Fy_N": Force_y_tire,
        "slip_ratios": slip_ratios,
        "slip_angles_rad": slip_angles,
        "ax_mps2": (Sum_Fx_body - drag - roll) / car.mass,
        "ay_mps2": Sum_Fy_body / car.mass,
    }

    dx= [x_rof, y_rof, yaw_rate,
      forward_velocity_ROF, lateral_velocity_ROF, yaw_rate_ROF,
      *wheel_angular_accel]

    return dx, outputs

