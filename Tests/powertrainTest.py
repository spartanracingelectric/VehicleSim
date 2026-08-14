import numpy as np
from Functions.loadConfigs import loadConfigs
from Vehicle.MainConfigs.SR17 import SR17
from Data.csvParser import load_speed_trace
from Data.plot import plot
from simrunner import SimRunner

batteryConfigs = loadConfigs("Vehicle.Battery.Configs")
TRACE_CSV = "Data/autox80kwh.csv"
NUM_LAPS = 1
VCU_commanded_torque_Nm = 231
RPM = 1000

def main():
    sim = SimRunner(SR17)
    run(sim)
    # Pack voltage history (time series). getCellVoltages_mV() is a
    # per-cell snapshot (len ≈ series_cells), not vs time.
    plot(
        sim.vehicle.battery.time_s,
        sim.vehicle.battery.soc_history,
        "Time (s)",
        "SOC (V)",
        "SOC vs Time",
    )

def run(sim: SimRunner) -> SimRunner:
    for i in range(len(sim.time_s)):
        sim.vehicle.inverter.update(
            VCU_commanded_torque_Nm, 
            sim.vehicle.motor.getMotor_kt(), 
            sim.vehicle.battery.getPackMaxCurrent_A(),
        )
        
        sim.vehicle.motor.update(
            sim.vehicle.inverter.getCurrent_A(),
            RPM,
        )
        
        sim.vehicle.battery.update(
            sim.vehicle.inverter.getCurrent_A(),
            sim.dt_s,
        )
        
        sim.vehicle.bms.update(
            sim.vehicle.battery.getCellVoltages_mV(),
            sim.vehicle.battery.getTemperatures_C(),
            sim.vehicle.battery.getShuntCurrent_mA(),
        )
    
if __name__ == "__main__":
    main()