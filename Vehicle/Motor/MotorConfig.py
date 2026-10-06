import math
import numpy as np
from Functions.clarkePark import clarke, park

class MotorConfig: #TODO: Add parameters related to motor
    # init function runs automatically when you create a MotorConfig object
    # the current controller lives in the inverter (InverterConfig.updateCurrentControl), this is just the motor's physics
    # maxCurrent_Arms, maxSpeed_rpm: None = no limit. fieldWeakeningCurrent_Arms: how much negative id
    # the inverter may use to weaken the field at high speed, 0 = no field weakening
    def __init__(self, Kt, Rs, Np, Lq, Ld, Jm, lambda_f, peakTorque_Nm,
                 maxCurrent_Arms=None, maxSpeed_rpm=None, fieldWeakeningCurrent_Arms=0.0):
        self.Kt = Kt #torque constant (Nm/A rms, datasheet)
        self.peakTorque_Nm = peakTorque_Nm #max torque at the motor shaft, before the gearbox
        # model currents are peak (amplitude), datasheet currents are rms
        self.maxCurrent_A = None if maxCurrent_Arms is None else maxCurrent_Arms * math.sqrt(2)
        self.maxSpeed_rpm = maxSpeed_rpm
        self.fieldWeakeningCurrent_A = fieldWeakeningCurrent_Arms * math.sqrt(2)
        # self.peak_power_W = peak_power_W
        self.Rs = Rs #stator winding resistance (ohm)
        self.Np = Np #number of poles
        self.Lq = Lq #quadrature axis inductance (H)
        self.Ld = Ld #direct axis inductance (H)
        self.Jm = Jm #inertia (kgm^2)
        self.lambda_f = lambda_f #rotor flux linkage (Wb)

        # initialize to 0 at beginning
        self.id = 0
        self.iq = 0
        self.wm = 0
        self.theta_m = 0
        self.Te = 0
        self.Pe = 0
        self.P_copper = 0 #copper loss in the windings (W)
        self.vd = 0 #last d/q voltages the motor saw, rotor frame (V)
        self.vq = 0


    # the inverter's three phase voltages go straight onto the motor's terminals.
    # turns them into the rotor frame (clarke, then park at the rotor's angle right now) and runs update
    # wm: pass the speed when something else sets it (the car through the gearbox, or a dyno),
    # leave it out and the motor spins up from its own inertia instead
    def update_from_phase_voltages(self, v_abc, delta_t, wm=None, T_load=0.0, Bm=0.0, T_friction=0.0):
        if wm is not None:
            self.set_wm(wm)

        theta_e = self.find_theta_e(self.theta_m)
        v_alpha, v_beta, _ = clarke(*v_abc)
        vd, vq = park(v_alpha, v_beta, theta_e)

        return self.update(vd, vq, delta_t, T_load, Bm, T_friction)


    # update the motor’s state using the current and speed
    # recieve vd and vq from inverter, delta t from running simulation
    # T_load, Bm, T_friction are the mechanical load on the shaft
    def update(self, vd, vq, delta_t, T_load=0.0, Bm=0.0, T_friction=0.0):
        self.vd = vd
        self.vq = vq
        we = self.find_we(self.wm)
        self.update_currents(vd, vq, we, delta_t)
        self.Te = self.find_Te()
        self.update_theta_m(delta_t)
        self.update_wm(self.Te, T_load, Bm, T_friction, delta_t)
        self.Pe = self.find_P_electrical(vd, vq)
        self.P_copper = self.find_P_copper()

        return self.id, self.iq, self.Te, self.wm, self.theta_m, self.Pe


    def find_we(self, old_wm):
         # (Np/2) number of pole pairs
        return (self.Np/2)*old_wm

    # electrical angle the inverter needs for the park transform
    def find_theta_e(self, theta_m):
        return ((self.Np/2) * theta_m) % (2 * math.pi)

    # q-axis current for a torque request with id = 0
    # uses the model's own torque equation, not the datasheet Kt (that one is per A rms)
    # with id from field weakening the (Ld - Lq)*id part of the torque equation counts too
    def find_iq_desired(self, torque_desired, id_A=0.0):
        return torque_desired / (1.5 * self.Np/2 * (self.lambda_f + (self.Ld - self.Lq) * id_A))

    # uses old id, old iq for both derivatives
    # then updates id and iq together
    def update_currents(self, vd, vq, we, delta_t):
        did_dt = (vd - (self.Rs * self.id) + (we * self.Lq * self.iq)) / self.Ld
        diq_dt = (vq - self.Rs * self.iq - we * (self.Ld * self.id + self.lambda_f)) / self.Lq
        self.id = self.id + did_dt*delta_t
        self.iq = self.iq + diq_dt*delta_t


    # uses updated id, updated iq
    def find_Te(self):
        Te = (1.5 * self.Np/2)* ((self.lambda_f * self.iq) + (self.Ld - self.Lq)*(self.id * self.iq))
        return Te


    def update_wm(self, Te, T_load, Bm, T_friction, delta_t):
        dwm_dt = (Te - T_load - Bm * self.wm - T_friction) / self.Jm
        self.wm = self.wm + dwm_dt*delta_t

    # uses old wm, so call before update_wm
    def update_theta_m(self, delta_t):
        self.theta_m = (self.theta_m + (self.wm * delta_t)) % (2 * math.pi)

    # for when something else sets the speed (dyno, or the vehicle through the gearbox)
    def set_wm(self, wm):
        self.wm = wm

    # inverter off (contactors open): nothing drives current through the windings, so no torque or power,
    # but the rotor keeps turning with the car. wm: the speed it's being turned at
    # (holds as long as the back-emf stays below what's left on the dc link capacitor)
    def turnOff(self, delta_t, wm=None):
        if wm is not None:
            self.set_wm(wm)
        self.update_theta_m(delta_t)

        self.id = 0
        self.iq = 0
        self.vd = 0
        self.vq = 0
        self.Te = 0
        self.Pe = 0
        self.P_copper = 0

    # heat in the windings from the current going through Rs, uses updated id, updated iq
    # Pe = Te*wm + P_copper (+ a little going in/out of the inductances while current changes)
    def find_P_copper(self):
        return 1.5 * self.Rs * (self.id**2 + self.iq**2)

    def find_P_electrical(self, vd, vq):
        return 1.5 * (vd * self.id + vq * self.iq)

    # returns motor constant, Kt
    def getMotor_kt(self):
        return self.Kt
