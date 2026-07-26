import numpy as np
import matplotlib.pyplot as plt
from Functions.loadConfigs import loadConfigs
from Vehicle.MainConfigs.SR16 import SR16

batteryConfigs = loadConfigs("Vehicle.Battery.Configs")

class SimRunner:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.rng = np.random.default_rng()
        self.dt_s = 1 #time step
        self.time_s = np.arange(0,100,self.dt_s) #TODO: import csv and get time vs speed trace 
        self.vel_mps = self.rng.random(len(self.time_s))
        self.accel_mps2 = np.zeros(len(self.time_s))
        self.force_N = np.zeros(len(self.time_s))
        self.torque_Nm = np.zeros(len(self.time_s))
        self.power_W = np.zeros(len(self.time_s))
        
def main():
    sim = SimRunner(SR16)
    run(sim)
    plot(sim)

def run(sim):
    vel_radps = sim.vel_mps / sim.vehicle.tire.radius_m
    batteries = list(vars(batteryConfigs).values())  # SimpleNamespace → list of BatteryConfig

    sim.battery_mass_type = [battery.mass_kg for battery in batteries]
    sim.total_power_KwHr = np.zeros(len(batteries))

    for idx, battery in enumerate(batteries):
        sim.vehicle.battery = battery  # swap pack onto SR16
        m_kg = sim.vehicle.vehicle_mass_kg + battery.mass_kg
        for i in range(len(sim.time_s) - 1):
            powerCalc(i, sim, m_kg, vel_radps)
        sim.total_power_KwHr[idx] = 22 * sum(sim.power_W) * sim.dt_s / (3600 * 1000)
    return sim

def powerCalc(i, sim, m_kg, vel_radps):
    sim.accel_mps2[i] = (sim.vel_mps[i+1]-sim.vel_mps[i])/(sim.dt_s)
    sim.force_N[i] = m_kg*sim.accel_mps2[i]
    sim.torque_Nm[i] = sim.force_N[i]*sim.vehicle.tire.radius_m
    sim.power_W[i] = sim.torque_Nm[i]*vel_radps[i]  
    
def plot(sim):
   # plt.plot(sim.total_power_KwHr, sim.battery_mass_type)
   # plt.show()
    print(sim.total_power_KwHr)
    print(sim.battery_mass_type)
if __name__ == "__main__":
    main()