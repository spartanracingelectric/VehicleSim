from copy import deepcopy

from Vehicle.Tire.TireConfig import TireConfig
from Vehicle.Tire.MagicFormulaTireConfig import MagicFormulaTireConfig

# one set of tire numbers, two models to pick from. edit the numbers in Configs/tire1.py, pick the model with
# the flag (--tire-model dugoff|pacejka in simrunner) or in code with tire.setModel("pacejka")
#
# notes:
# - dugoff only uses radius, mu and the two stiffnesses. pacejka also uses the C and E shape numbers
# - the models remember their last slip and force (and filter state), so every wheel needs its own copy.
#   createWheelModels(4) gives 4 fresh ones, the chassis does this for you
# - force_filter_time_constant_s is the optional first order filter on the tire forces, 0 = off (default).
#   only turn it on if the forces oscillate, a few ms (0.002 - 0.005) is a start. the chassis also has a
#   slip filter (slip_filter_time_constant_s), try that one first
class Tire:
    MODELS = ("dugoff", "pacejka")

    def __init__(self, radius_m, mu, long_stiff_per_load, lat_stiff_per_load, C_long=1.5, E_long=0.0, C_lat=1.3,
                 E_lat=0.0, model="dugoff", force_filter_time_constant_s=0.0):
        if radius_m <= 0:
            raise ValueError(f"radius_m must be > 0, got {radius_m}")
        self.radius_m = radius_m
        self.mu = mu                                        # tire road friction, force limit = mu * Fz
        self.long_stiff_per_load = long_stiff_per_load      # Cx / Fz, longitudinal slip stiffness
        self.lat_stiff_per_load = lat_stiff_per_load        # Cy / Fz, cornering stiffness (1/rad)
        self.C_long = C_long                                # pacejka only, shape
        self.E_long = E_long                                # pacejka only, peak sharpness
        self.C_lat = C_lat
        self.E_lat = E_lat
        self.force_filter_time_constant_s = force_filter_time_constant_s
        self.model = None
        self.setModel(model)
        self._default_model = None

    def setModel(self, model):
        name = {"dugoff": "dugoff", "pacejka": "pacejka", "magic_formula": "pacejka"}.get(str(model).lower())
        if name is None:
            raise ValueError(f"tire model must be one of {self.MODELS}, got {model!r}")
        self.model = name
        self._default_model = None
        return self

    # model_name uses the old names (dugoff / magic_formula) that the model classes have
    @property
    def model_name(self):
        return "dugoff" if self.model == "dugoff" else "magic_formula"

    def createModel(self):
        if self.model == "dugoff":
            return TireConfig(self.radius_m, longitudinal_stiffness_per_load=self.long_stiff_per_load,
                              lateral_stiffness_per_load=self.lat_stiff_per_load, tireroadfriction=self.mu,
                              force_filter_time_constant_s=self.force_filter_time_constant_s)
        return MagicFormulaTireConfig(self.radius_m, self.mu, self.C_long, self.E_long, self.long_stiff_per_load,
                                      self.C_lat, self.E_lat, self.lat_stiff_per_load,
                                      force_filter_time_constant_s=self.force_filter_time_constant_s)

    def createWheelModels(self, count=4):
        return [self.createModel() for _ in range(count)]

    # for single tire use (plots, tests): one shared model, don't use this for 4 wheels
    def getModel(self):
        if self._default_model is None:
            self._default_model = self.createModel()
        return self._default_model

    def calculateForces(self, tireDownForce, slip_ratio, slip_angle, dt_s=None):
        return self.getModel().calculateForces(tireDownForce, slip_ratio, slip_angle, dt_s=dt_s)

    def isAtGripLimit(self):
        return self.getModel().isAtGripLimit()

    def copy(self):
        return deepcopy(self)
