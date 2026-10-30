import math
# ackerman placeholder is 0 for now, we need to update ackerman ratio
class SteeringConfig:
    def __init__(self, wheelbase_m, track_m, ackerman_fraction ):
        self.wheelbase_m = wheelbase_m
        self.track_m = track_m
        self.ackerman_fraction = ackerman_fraction

    def getWheelAngles_rad(self, delta_rad):

        if delta_rad == 0:
             return 0.0, 0.0

        R_turn = self.wheelbase_m / (math.tan(abs(delta_rad)))

        # inside and outside are how far each wheel is turned from pointing straight ahead
        inside = math.atan(self.wheelbase_m / (R_turn - self.track_m/2))
        outside = math.atan(self.wheelbase_m / (R_turn + self.track_m/2))

        inside_blended = abs(delta_rad) + self.ackerman_fraction * (inside - abs(delta_rad))
        outside_blended = abs(delta_rad) + self.ackerman_fraction * (outside - abs(delta_rad))

        # left turn
        if delta_rad>0: 
             return inside_blended, outside_blended
        
        else:
            return -outside_blended, -inside_blended
        


        
        


        
    


