import matplotlib.pyplot as plt
from Vehicle.Tire.TireConfig2 import TireConfig

#TODO: 
#get the right coefficients for c_long/lat/tiremu dependent on load
#Create a for loop on Fz to test tire load sensitivity
#figure out how to import some stuff from another file for Fz, Vx, Vy, omega, etc
#split latmu and longmu

#Assumptions:
#4 deg static camber, 8psi tires parsed from TTC 667N
#NO TIRE LOAD SENSITIVITY   
#16" longitudinal tire behavior extrapolated from 20" & 18" R20 behavior

tire1 = TireConfig(
    radius_m = 0.4064,
    c_long = 25.691, #cornering stiffness N/rad divided by normal load
    c_lat = 33.959, #longitudinal stiffnes N/rad divided by normal load
    tiremu = 1.5
)
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

