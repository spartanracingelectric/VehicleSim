import numpy
class MagicFormulaTireConfig: #TODO: load sensitivity (mu changing with Fz), camber, aligning moment

    # Magic Formula (Pacejka), same curve for both directions, just its own numbers each way
    # Fx = D * sin(C * atan(B*k - E*(B*k - atan(B*k))))          k is the slip ratio
    # Fy = D * sin(C * atan(B*a - E*(B*a - atan(B*a))))          a is the slip angle (rad)

    # D is maximum force, that is F = (u * Fz)
    # C is shape, how much force is left when the tire is fully sliding, sin(C * pi / 2) of the peak
    #   (1.5 leaves 71%), has to be between 1 and 2: 1 or less has no peak, 2 or more pushes backwards when sliding
    # B is stiffness factor, it's chosen so that B * C * D = your slip stiffness, so B = (25.691 * Fz) / (C * D)
    #   the Fz cancels out (no load sensitivity yet), so B is the same at every load
    # E is how sharp the peak is, starting with 0, can be negative but not over 1 (the curve folds back on itself)
    #
    # combined slip (braking/driving while cornering) shares the grip by normalized slip:
    # each slip is divided by the slip where its force peaks, kn = k / k_peak, an = a / a_peak, rho = hypot(kn, an)
    # the tire acts like it's at rho of the way to the peak in both directions and splits the force kn : an
    # at rho = 1 the tire is at its peak, past that it's sliding and the force drops off
    #
    # same calls as the dugoff TireConfig: calculateForces(Fz, k, a) -> (Fx, Fy), isAtGripLimit(), radius_m
    # slip conventions are the same too: k > 0 driving, k < 0 braking, locked = -1, Fy has the sign of a

    model_name = "magic_formula"

    tireSlipRatio = 0.0 # longitudinal slip ratio
    tireSlipAngle = 0.0 # lateral slip angle (rad)
    tireDownForce = 0.0 # downward force on the tire
    tireLongitudinalForce = 0.0 # longitudinal force on the tire
    tireLateralForce = 0.0 # lateral force on the tire
    tireNormalizedSlip = 0.0 # rho, 1 means the tire is at the peak of its curve

    def __init__(self, radius_m, mu, C_long, E_long, long_stiff_per_load, C_lat, E_lat, lat_stiff_per_load):
        if radius_m is None or radius_m <= 0:
            raise ValueError(f"radius_m must be > 0, got {radius_m}")
        if mu < 0:
            raise ValueError(f"mu can't be negative, got {mu}")
        for name, C in (("C_long", C_long), ("C_lat", C_lat)):
            if not 1 < C < 2:
                raise ValueError(f"{name} (shape) must be between 1 and 2, got {C}")
        for name, E in (("E_long", E_long), ("E_lat", E_lat)):
            if E > 1:
                raise ValueError(f"{name} (peak sharpness) can't be over 1, got {E}")
        for name, stiffness in (("long_stiff_per_load", long_stiff_per_load), ("lat_stiff_per_load", lat_stiff_per_load)):
            if stiffness <= 0:
                raise ValueError(f"{name} must be > 0, got {stiffness}")

        self.radius_m = radius_m # radius of the tire in meters
        self.mu = mu # friction between tire and road, D = mu * Fz
        self.C_long = C_long
        self.E_long = E_long
        self.long_stiff_per_load = long_stiff_per_load # longitudinal slip stiffness / Fz, B*C*D = this * Fz
        self.C_lat = C_lat
        self.E_lat = E_lat
        self.lat_stiff_per_load = lat_stiff_per_load # cornering stiffness / Fz (1/rad)

        # slip where each curve peaks, doesn't change with load (B doesn't) so it's worked out once
        self.peak_slip_ratio = self.getPeakSlip(self.getB(self.long_stiff_per_load, self.C_long), self.C_long, self.E_long)
        self.peak_slip_angle = self.getPeakSlip(self.getB(self.lat_stiff_per_load, self.C_lat), self.C_lat, self.E_lat)

    def calculateForces(self, tireDownForce, slip_ratio, slip_angle):
        # One call for the chassis, same as the dugoff TireConfig.calculateForces
        self.tireSlipRatio = slip_ratio
        self.tireSlipAngle = slip_angle
        self.tireDownForce = tireDownForce
        self.tireLongitudinalForce = self.calculateTireLongitudinalForce(tireDownForce)
        self.tireLateralForce = self.calculateTireLateralForce(tireDownForce)
        return self.tireLongitudinalForce, self.tireLateralForce

    # k is the wheel slip ratio, uses the stored slip ratio and slip angle (combined slip)
    def calculateTireLongitudinalForce(self, Fz):
        normalized_slip_ratio, normalized_slip_angle, rho = self.getNormalizedSlip()
        if Fz <= 0 or rho == 0:
            return 0.0 # no load or no slip, no force
        pure_force = self.getPureForce(Fz, rho * self.peak_slip_ratio, self.long_stiff_per_load, self.C_long, self.E_long)
        return normalized_slip_ratio / rho * pure_force

    # a is the slip angle, uses the stored slip ratio and slip angle (combined slip)
    def calculateTireLateralForce(self, Fz):
        normalized_slip_ratio, normalized_slip_angle, rho = self.getNormalizedSlip()
        if Fz <= 0 or rho == 0:
            return 0.0
        pure_force = self.getPureForce(Fz, rho * self.peak_slip_angle, self.lat_stiff_per_load, self.C_lat, self.E_lat)
        return normalized_slip_angle / rho * pure_force

    def getPureForce(self, Fz, slip, stiff_per_load, C, E):
        # the magic formula itself, one direction only (no slip the other way)
        if Fz <= 0:
            return 0.0
        D = self.mu * Fz
        B = self.getB(stiff_per_load, C)
        return D * numpy.sin(C * numpy.arctan(B * slip - E * (B * slip - numpy.arctan(B * slip))))

    def getB(self, stiff_per_load, C):
        # B = stiffness / (C * D) = (stiff_per_load * Fz) / (C * mu * Fz), the Fz cancels
        return stiff_per_load / (C * self.mu)

    def getPeakSlip(self, B, C, E):
        # the curve peaks when C * atan(x) = pi / 2, so x = B*s - E*(B*s - atan(B*s)) = tan(pi / (2C))
        # x only goes up with s (E <= 1), so halve the gap until it's found (bisection) instead of solving by hand
        target = numpy.tan(numpy.pi / (2 * C))
        low, high = 0.0, 1.0
        while high * B - E * (high * B - numpy.arctan(high * B)) < target:
            high *= 2
        for _ in range(60):
            middle = (low + high) / 2
            if middle * B - E * (middle * B - numpy.arctan(middle * B)) < target:
                low = middle
            else:
                high = middle
        return (low + high) / 2

    def getNormalizedSlip(self):
        # each slip as a fraction of where its curve peaks, and how far along the combined slip is
        normalized_slip_ratio = self.tireSlipRatio / self.peak_slip_ratio
        normalized_slip_angle = self.tireSlipAngle / self.peak_slip_angle
        self.tireNormalizedSlip = numpy.hypot(normalized_slip_ratio, normalized_slip_angle)
        return normalized_slip_ratio, normalized_slip_angle, self.tireNormalizedSlip

    def isAtGripLimit(self):
        # at or past the peak of the curve after the last calculateForces (more slip gives less force from here)
        return self.tireDownForce > 0 and self.tireNormalizedSlip >= 1.0

    # same as TireConfig (dugoff)
    def calculateTireSlipRatio(self, wheel_speed, vehicle_speed):
        # driving > 0, braking < 0, locked wheel = -1
        self.tireSlipRatio = (wheel_speed - vehicle_speed) / vehicle_speed if vehicle_speed != 0 else 0.0
        return self.tireSlipRatio

    # same as TireConfig (dugoff)
    def calculateTireSlipAngle(self, wheel_angle, wheel_longitudinal_velocity, wheel_lateral_velocity):
        self.tireSlipAngle = wheel_angle - numpy.arctan(wheel_lateral_velocity / wheel_longitudinal_velocity) if wheel_longitudinal_velocity != 0 else 0.0
        return self.tireSlipAngle

    def getTireSlipRatio(self):
        return self.tireSlipRatio
    def getTireSlipAngle(self):
        return self.tireSlipAngle
