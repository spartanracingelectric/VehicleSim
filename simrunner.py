import numpy as np
from Functions.loadConfigs import loadConfigs
from Vehicle.MainConfigs.SR17 import SR17
from Data.csvParser import load_speed_trace
from Data.plot import plot

batteryConfigs = loadConfigs("Vehicle.Battery.Configs")
TRACE_CSV = "Data/autox80kwh.csv"
NUM_LAPS = 1

class SimRunner:
    def __init__(self, vehicle, csv_path=TRACE_CSV):
        self.vehicle = vehicle
        # self.time_s, self.vel_mps = load_speed_trace(csv_path)
        self.time_s = np.arange(0, 100, 1)
        self.dt_s = float(np.mean(np.diff(self.time_s)))
        self.accel_mps2 = np.zeros(len(self.time_s))
        self.force_N = np.zeros(len(self.time_s))
        self.torque_Nm = np.zeros(len(self.time_s))
        self.power_W = np.zeros(len(self.time_s))
        
def main():
    sim = SimRunner(SR17)
    run(sim)
    plot(sim)

def run(sim: SimRunner) -> SimRunner:
    vel_radps = sim.vel_mps / sim.vehicle.tire.radius_m
    batteries = list(vars(batteryConfigs).values())

    sim.battery_mass_type = [battery.mass_kg for battery in batteries]
    sim.total_power_KwHr = np.zeros(len(batteries))

    for idx, battery in enumerate(batteries):
        sim.vehicle.battery = battery
        m_kg = sim.vehicle.vehicle_mass_kg + battery.mass_kg
        for i in range(len(sim.time_s) - 1):
            powerCalc(i, sim, m_kg, vel_radps)
        drive_power_W = np.maximum(sim.power_W, 0.0)
        sim.total_power_KwHr[idx] = NUM_LAPS * np.sum(drive_power_W) * sim.dt_s / (3600 * 1000)
    return sim

def powerCalc(i: int, sim: SimRunner, m_kg: float, vel_radps: np.ndarray) -> None:
    sim.accel_mps2[i] = (sim.vel_mps[i+1]-sim.vel_mps[i])/(sim.dt_s)
    sim.force_N[i] = m_kg*sim.accel_mps2[i]
    sim.torque_Nm[i] = sim.force_N[i]*sim.vehicle.tire.radius_m
    sim.power_W[i] = sim.torque_Nm[i]*vel_radps[i]  
    
if __name__ == "__main__":
    main()