import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))    # repo root, so this also runs as a file or with the run button
import matplotlib.pyplot as plt
from Vehicle.Tire.Configs.tire1 import tire1

# run: python Tests/tireTest.py (or python -m Tests.tireTest from the repo root)
# (moved here from Vehicle/Tire/Configs/tiretesting2.py, the tire itself is now Configs/tire1.py)

#TODO:
#get the right coefficients for c_long/lat/tiremu dependent on load
#Create a for loop on Fz to test tire load sensitivity
#figure out how to import some stuff from another file for Fz, Vx, Vy, omega, etc
#split latmu and longmu

kappa, alpha = tire1.calculateslip(
    Vx = 25, #vehicle speed 25 m/s
    Vy = 0, #lat velocity (m/s)
    omega = 64.6, #tire rotation speed (rad/s) (64.6 * 0.4064 = 26.25 m/s)
    delta = 0.197 #Steer angle (rad) 0.197 rad = 11.3 deg
)

kappa_values = [] #initializing indexing for kappa & Fx
Fx_values = []
#alpha_values = []
#Fy_values = []

for i in range(-50,51): 
    kappa = i/100 #creates kappa from -0.5 to 0.50 with 0.01 steps
    kappahat, alphahat, shat, Fres, Fx, Fy = tire1.calculateforces(
        Fz = 667, #667 newtons
        kappa = kappa, #required to set  for indexing kappa & Fx
        alpha = 0, #set to 0 for now because straight line iterations
    )
    kappa_values.append(kappa)
    Fx_values.append(Fx)
    #alpha_values.append(alpha)
    #Fy_values.append(Fy)

print(kappa_values)


#plots kappa vs Fx with MATLAB!!
plt.plot(kappa_values, Fx_values)

plt.xlabel("Slip Ratio, kappa")
plt.ylabel("Longitudinal Force, Fx [N]")
plt.title("Dugoff Fx vs Slip Ratio")

plt.grid()
plt.show()    

#print(tire1.c_lat)
#print("Fx is",Fx)   
#print("kappa is", kappa)
#print("kappahat is", kappahat)
#print("alpha is", alpha)
#print("alphahat is", alphahat)
#print("shat is", shat)
#print("Fres is", Fres)
#rint("Fx is", Fx)
#print("Fy is", Fy)  

