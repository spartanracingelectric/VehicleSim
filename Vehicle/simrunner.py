import numpy as np
import Configs.SR16 as SR16
import matplotlib.pyplot as plt
import random

def update(i):
    accel_mps2[i] = (vel_mps[i+1]-vel_mps[i])/(dt_s)
    force_N[i] = m_kg*accel_mps2[i]
    torque_Nm[i] = force_N[i]*SR16.TIRE_RADIUS_M
    power_W[i] = torque_Nm[i]*vel_radps[i]  
    
rng = np.random.default_rng()
m_kg = SR16.VEHICLE_MASS_KG + SR16.PACK_MASS_KG
dt_s = 1 #time step
time_s = np.arange(0,100,dt_s) #TODO: import csv and get time vs speed trace 
vel_mps = rng.random(len(time_s))
accel_mps2 = np.zeros(len(time_s))
force_N = np.zeros(len(time_s))
torque_Nm = np.zeros(len(time_s))
vel_radps = vel_mps/SR16.TIRE_RADIUS_M
power_W = np.zeros(len(time_s))
pack_mass_type = [SR16.PACK_MASS_KG, SR16.PACK_MASS_KG-5, SR16.PACK_MASS_KG+5]
total_power_KwHr = np.zeros(3)


for pack_mass in range(len(pack_mass_type)):
    m_kg = SR16.VEHICLE_MASS_KG + pack_mass_type[pack_mass]
    for i in range(len(time_s)-1):

        accel_mps2[i] = (vel_mps[i+1]-vel_mps[i])/(dt_s)
        force_N[i] = m_kg*accel_mps2[i]
        torque_Nm[i] = force_N[i]*SR16.TIRE_RADIUS_M
        power_W[i] = torque_Nm[i]*vel_radps[i]
    total_power_KwHr[pack_mass] = 22*sum(power_W)*dt_s/(3600*1000)

print(total_power_KwHr)
print(pack_mass_type)

plt.plot(total_power_KwHr, pack_mass_type)
plt.show()