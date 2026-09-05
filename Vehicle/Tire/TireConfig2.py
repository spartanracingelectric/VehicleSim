import math
class TireConfig: 

#TODO:
#replace c_long in def calculateforces with a get_cornering_stiffness object to extrapolate from normal load

    def __init__(self, radius_m, c_long, c_lat, tiremu):
        self.radius_m = radius_m #tire radius in meters
        self.c_long = c_long #normalized longitudinal stiffness coeff
        self.c_lat = c_lat #normalized lateral stiffness coeff
        self.tiremu = tiremu #tire/road friction coeff
        

    def calculateslip(self, Vx, Vy, omega, delta):
       
        kappa = (omega * self.radius_m - Vx) / (omega * self.radius_m)  #slip ratio
        kappa = max(-0.999, min(0.999, kappa)) #limit slip ratio to +-1

        alpha = delta - math.atan2(Vy, Vx) #slip angle (deg)

        return kappa, alpha

    
    def calculateforces(self, Fz, kappa, alpha):

        Cs = self.c_long * Fz  #long cornering stiffness at Fz normal load
        Ca = self.c_lat * Fz #lat cornering stiffness at Fz normal load
        D = self.tiremu * Fz #tire force scale with normal load

        kappahat = ( kappa * (Cs / D) ) / ( 1 - kappa) #normalized longitudinal slip

        alphahat = ( math.tan(alpha) * Ca/D ) / ( 1 - kappa ) #normalized lateral slip

        shat = math.sqrt( alphahat **2 + kappahat**2 )

        #normalized slip for saturation calcs
        if shat == 0: 
            Fres = 0
            Fx = 0
            Fy = 0
        elif shat < 0.5:
            Fres = D * shat
            Fx = (kappahat / shat) * Fres
            Fy = (alphahat / shat) * Fres
        else:
            Fres = D * (1 - (1 / (4 * shat)))
            Fx = (kappahat / shat) * Fres
            Fy = (alphahat / shat) * Fres
    

        
        
        return kappahat, alphahat, shat, Fres, Fx, Fy
       