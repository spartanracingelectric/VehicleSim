from Functions.PIDcontroller import PIDController
import math


class RearTVConfig:
    def __init__(self, kp_Nm_per_radps, ki_Nm_per_rad, wheelbase_m, ref_filter_time_s, understeer_gradient=0.0, mode="inside_cut", torque_rate_limit_Nm_per_s=None):


        self.kp_Nm_per_radps = kp_Nm_per_radps
        self.ki_Nm_per_rad = ki_Nm_per_rad
        self.wheelbase_m = wheelbase_m
        self.understeer_gradient = understeer_gradient #a number, or a function of speed that gives K_us at each speed
        self.ref_filter_time_s = ref_filter_time_s
        self.mode = mode
        self.torque_rate_limit_Nm_per_s = torque_rate_limit_Nm_per_s #None means the cut can jump any amount in one update

        # things taht change every update
        self.tv_pi = PIDController(Kp=kp_Nm_per_radps, Ki=ki_Nm_per_rad, output_min=0.0)
        self.r_ref_filtered_radps = None
        self.previous_cut_Nm = 0.0

        # logging: filled in by each update so the runner can record them
        self.r_ref_radps = 0.0
        self.yaw_error_radps = 0.0
        self.inside_cut_Nm = 0.0
        self.is_saturated = False

        
    def update(self, x, delta_rad, base_wheel_torque_Nm, dt_s):

        if delta_rad == 0:
            return [base_wheel_torque_Nm] * 4, 0.0
         


        
        current_speed = x[3] #from state vector
        if callable(self.understeer_gradient):
            understeer_gradient = self.understeer_gradient(current_speed)
        else:
            understeer_gradient = self.understeer_gradient
        r_ref = current_speed*delta_rad/(self.wheelbase_m + understeer_gradient * current_speed**2)

        
        #sometimes the desired yaw rate might not be acheivable in one step. To combat this, we must find a more manageable yaw rate to target
        if self.r_ref_filtered_radps is None:
            self.r_ref_filtered_radps = r_ref 
        
        self.r_ref_filtered_radps += dt_s/ self.ref_filter_time_s * (r_ref - self.r_ref_filtered_radps)


        r = x[5] #current yaw rate
        if delta_rad>0:
            turn_sign = 1

        else:
            turn_sign = -1

        e_turn = turn_sign * (self.r_ref_filtered_radps - r)

        self.r_ref_radps = r_ref
        self.yaw_error_radps = e_turn
        self.inside_cut_Nm = 0.0
        self.is_saturated = False

        

        if self.mode == "off":
            return [base_wheel_torque_Nm] * 4, 0.
        
        if self.mode == "oracle":
            self.tv_pi.output_min = None
            self.tv_pi.output_max = None
            moment = turn_sign * self.tv_pi.update(e_turn, dt_s)
            return [base_wheel_torque_Nm] * 4, moment
        

        self.tv_pi.output_max = base_wheel_torque_Nm
        torques = [base_wheel_torque_Nm] *4
        torque_cut = self.tv_pi.update(e_turn, dt_s)

        #the motor can't change torque instantly, so the cut can only move so far from last update's cut
        if self.torque_rate_limit_Nm_per_s is not None:
            max_change = self.torque_rate_limit_Nm_per_s * dt_s
            torque_cut = min(max(torque_cut, self.previous_cut_Nm - max_change), self.previous_cut_Nm + max_change)
            torque_cut = min(torque_cut, base_wheel_torque_Nm)
        self.previous_cut_Nm = torque_cut

        if delta_rad>0:
            inside = 2
        
        else:
            inside = 3
        

        torques[inside] -= torque_cut

        
        self.inside_cut_Nm = torque_cut
        self.is_saturated = (torque_cut == base_wheel_torque_Nm)
        return torques, 0
        

        



        
        


