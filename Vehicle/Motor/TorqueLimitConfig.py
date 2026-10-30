import numpy as np
# Vehicle/Motor/TorqueLimitConfig.py, Configs/amk_limits.py

class TorqueLimitConfig:
    def __init__(self, rpm_points, torque_points_Nm, gear_ratio, power_limit_W, efficiency=1.0):
        self.rpm_points = rpm_points
        self.torque_points_Nm = torque_points_Nm
        self.gear_ratio = gear_ratio
        self.power_limit_W = power_limit_W
        self.efficiency = efficiency


    def getMaxWheelTorque_Nm(self, wheel_speed_rad_per_second):
        motor_radps = wheel_speed_rad_per_second * self.gear_ratio
        motor_rpm = (motor_radps * 60)/(2*np.pi)
        # motor torque times gear ratio equals wheel torque
        return float(np.interp(motor_rpm, self.rpm_points, self.torque_points_Nm, right = 0.0)) * self.gear_ratio 

    def capWheelTorques_Nm(self, wheel_torques_Nm, wheel_speeds_radps_arr):

        total_power = 0
        capped_torques = []

        for x in range(len(wheel_speeds_radps_arr)):
            max_torque = self.getMaxWheelTorque_Nm(wheel_speeds_radps_arr[x])
            capped_torques.append(max(0.0, min(wheel_torques_Nm[x],max_torque)))
            total_power += (capped_torques[x])*(wheel_speeds_radps_arr[x])

        
        # checking whether total power (energy delivered per second to wheel) 
        # is below 80kW, must be under rules and regulations

    

        if total_power / self.efficiency > self.power_limit_W:
            scale = self.power_limit_W * self.efficiency / total_power
            capped_torques = [t * scale for t in capped_torques]

        

        return capped_torques



        



