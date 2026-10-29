from Functions.constants import AIR_DENSITY_KGPM3

class AeroConfig:
    def __init__(self, lift_area_m2, drag_area_m2, rear_downforce_fraction):

        self.lift_area_m2 = lift_area_m2
        self.drag_area_m2  = drag_area_m2
        self.rear_downforce_fraction = rear_downforce_fraction

    
    def getDownforce_N(self, u_mps):
        return 0.5* AIR_DENSITY_KGPM3 * self.lift_area_m2 * u_mps**2
    
    def getRearDownforce_N(self, u_mps):
        return self.getDownforce_N(u_mps) * self.rear_downforce_fraction
    
    def getFrontDownforce_N(self, u_mps):
        return self.getDownforce_N(u_mps) * (1-self.rear_downforce_fraction)
    
    def getDrag_N(self,u_mps):
        return 0.5* AIR_DENSITY_KGPM3 * self.drag_area_m2 * u_mps**2