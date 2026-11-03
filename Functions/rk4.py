
from Vehicle.Chassis.PlanarChassis import derivatives


def rk4Step(x, inputs, car, dt_s, f=derivatives):
    k1, outputs= f(x, inputs, car)
    k2, _= f([x[i] + 0.5 * dt_s * k1[i] for i in range(len(x))], inputs, car)
    k3, _= f([x[i] + 0.5 * dt_s * k2[i] for i in range(len(x))], inputs, car)
    k4, _= f([x[i] + dt_s * k3[i] for i in range(len(x))], inputs, car)

    x_next= [x[i] + (dt_s / 6.0) * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]) for i in range(len(x))]

    return x_next, outputs