from Functions.PIDcontroller import PIDController
import math

class SkidPadDriverConfig: 
    def __init__(self, mode, target_speed_mps,  radius_m , turn, lookahead_m, wheelbase_m, speed_kp, speed_ki, max_wheeltorque_Nm, delta_rad = None, center_m = None):
        self.mode = mode
        self.target_speed_mps = target_speed_mps #target speed mps is a function of time, gives us target speed value at each time 
        self.delta_red = delta_rad
        self.center_m = center_m
        self.radius_m = radius_m
        self.turn = turn
        self.lookahead_m = lookahead_m
        self.wheelbase_m = wheelbase_m
        self.speed_kp = speed_kp
        self.speed_ki = speed_ki
        self.max_wheeltorque_Nm = max_wheeltorque_Nm
        self.speed_pi = PIDController(Kp = self.speed_kp, Ki = self.speed_ki , output_min  = 0 , output_max = self.max_wheeltorque_Nm)

    

    #update function is handling updating TORQUE COMMAND and STEERING 


    #STEERING CAN BE USED IN TWO MODES 

    #mode 1: self.mode = "fixed"
    #This mode fixes the way one steers, but allows you to change the path/sharpness of turns, but steering angle stays the same for all


    #mode 2: self.mode = "circle"
    #Uses a PURE PURSUIT algorithm to be able to change steering angle, circular path's radius is set constant here to the same dimensions of skidpad event

    def update(self, x, time_s, dt_s):
        target = self.target_speed_mps(time_s)
        u = x[3] #x is a state vector, and it has an entry which is current speed at index 3
        error_speed = target - u
        torque_command  = self.speed_pi.update(error_speed, dt_s)




        if self.mode == "fixed":
            delta = self.delta_rad(time_s)



        
        elif self.mode=="circle":
            cx,cy = self.center_m
            X,Y = x[0], x[1]
            theta_car = math.atan2(Y-cy, X-cx) 

            if self.turn == "left":
                sign = 1

            else: 
                sign = -1
                
            

            theta_target = theta_car + sign*2*math.asin(self.lookahead_m/(2*self.radius_m))


            Tx = cx + self.radius_m * math.cos(theta_target)
            Ty = cy + self.radius_m * math.sin(theta_target)

            psi = x[2] #heading angle

            eta = math.atan2(Ty - Y, Tx - X) - psi
            eta = (eta + math.pi) % (2 * math.pi) - math.pi
            
            delta = math.atan(2 * self.wheelbase_m * math.sin(eta) / self.lookahead_m)





            return delta, torque_command

            

        

        