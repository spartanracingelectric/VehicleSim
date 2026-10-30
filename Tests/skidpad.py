import math
import numpy as np
from Functions.rk4 import rk4Step




def runSkidpad(car, driver, tv, run_time_s, start_speed_mps=5.0, dt_s=0.0005, control_period_s=0.01):
        
        x, center = car.getStartStateOnCircle(start_speed_mps, driver.radius_m, driver.turn)
        t = 0.0
        ax, ay = 0.0, 0.0
        log = []
        stop_reason = "time_up"

        steps_per_tick = round(control_period_s / dt_s)
        num_ticks = round(run_time_s / control_period_s)
        
        for tick in range (num_ticks):
        
        
            delta, base_torque = driver.update(x, t, control_period_s)                  
            d_FL, d_FR = car.steering.getWheelAngles_rad(delta)                         
            torques, yaw_moment = tv.update(x, delta, base_torque, control_period_s)    
            torques = car.limits.capWheelTorques_Nm(torques, x[6:10])                   

            inputs = {
                "steer_angles_rad": [d_FL, d_FR, 0.0, 0.0],
                "wheel_torques_Nm": torques,
                "external_yaw_moment_Nm": yaw_moment,
                "ax_mps2": ax,
                "ay_mps2": ay,
            }

            for step in range(steps_per_tick):
                    x, outputs = rk4Step(x, inputs, car, dt_s)

                    inputs["ax_mps2"] = outputs["ax_mps2"]
                    inputs["ay_mps2"] = outputs["ay_mps2"]
                    ax, ay = inputs["ax_mps2"], inputs["ay_mps2"]
                
            

                
            inside = 2 if delta > 0 else 3
            outside = 3 if delta > 0 else 2
            rear_half_track = car.getTrack(3) / 2


            tv_yaw_moment = (outputs["Fx_N"][outside] - outputs["Fx_N"][inside]) * rear_half_track
            motor_power = sum(torques[i] * x[6 + i] for i in range(4))

            
            log.append({
                "time_s": t,
                "X_m": x[0], "Y_m": x[1], "psi_rad": x[2],
                "u_mps": x[3], "v_mps": x[4], "r_radps": x[5],
                "sideslip_rad": math.atan2(x[4], x[3]),
                "ay_mps2": ay,
                "delta_rad": delta,
                "r_ref_radps": tv.r_ref_radps,
                "r_ref_filtered_radps": tv.r_ref_filtered_radps,
                "yaw_error_radps": tv.yaw_error_radps,
                "inside_cut_Nm": tv.inside_cut_Nm,
                "is_saturated": tv.is_saturated,
                "external_yaw_moment_Nm": yaw_moment,
                "tv_yaw_moment_Nm": tv_yaw_moment,                
                "motor_power_W": motor_power,  
                "wheel_torques_Nm": list(torques),
                "Fz_N": list(outputs["Fz_N"]),
                "Fx_N": list(outputs["Fx_N"]),
                "Fy_N": list(outputs["Fy_N"]),
                "slip_ratios": list(outputs["slip_ratios"]),
                "slip_angles_rad": list(outputs["slip_angles_rad"]),
                "radius_error_m": math.hypot(x[0] - center[0], x[1] - center[1]) - driver.radius_m,
            })



            if x[3] < 3.0:
                stop_reason = "too slow"
                break
            if abs(log[-1]["radius_error_m"]) > 2.0:
                stop_reason = "off the circle"
                break

            if abs(log[-1]["sideslip_rad"]) > math.radians(20):
                stop_reason = "spun"
                break





                        
            t += control_period_s

        result = {}
        for name in log[0]:                                          
            result[name] = np.array([row[name] for row in log])     
        result["stop_reason"] = stop_reason
        result["center_m"] = center
        return result


