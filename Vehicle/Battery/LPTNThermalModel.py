import math
from copy import deepcopy
from Vehicle.Battery.TractiveBatteryPack import TractiveBatteryPack
import numpy as np


class LPTNThermalModel:
    # two temperatures: sensor and pack
    def __init__(
        self,
        c_pack_j_per_c=30000.0,
        c_sensor_j_per_c=5657.0,
        r_link_c_per_w=0.00468,
        r_th_c_per_w=0.312,
        q_i2_coeff=0.500,
        q_iabs_coeff=0.0,
        q_bias_w=0.0,
    ):
        heat_capacities_and_resistances = (
            c_pack_j_per_c,
            c_sensor_j_per_c,
            r_link_c_per_w,
            r_th_c_per_w,
        )
        if any(not math.isfinite(value) or value <= 0
               for value in heat_capacities_and_resistances):
            raise ValueError("Heat capacities and thermal resistances must be finite and positive")

        heat_coefficients = (q_i2_coeff, q_iabs_coeff, q_bias_w)
        if any(not math.isfinite(value) or value < 0 for value in heat_coefficients):
            raise ValueError("Heat coefficients must be finite and nonnegative")

        self.c_pack_j_per_c = c_pack_j_per_c
        self.c_sensor_j_per_c = c_sensor_j_per_c
        self.r_link_c_per_w = r_link_c_per_w
        self.r_th_c_per_w = r_th_c_per_w
        self.q_i2_coeff = q_i2_coeff
        self.q_iabs_coeff = q_iabs_coeff
        self.q_bias_w = q_bias_w
        self.temperature_label = "SR17 BMS fault-average proxy"

        # heat transfer between the two temperatures
        sensor_link_rate = 1 / (c_sensor_j_per_c * r_link_c_per_w)
        pack_link_rate = 1 / (c_pack_j_per_c * r_link_c_per_w)
        pack_cooling_rate = 1 / (c_pack_j_per_c * r_th_c_per_w)
        temperature_rates = np.array([
            [-sensor_link_rate, sensor_link_rate],
            [pack_link_rate, -pack_link_rate - pack_cooling_rate],
        ])

        # separate the two cooling rates so each time step can be exact.
        # the rates are always real here. numpy 2 returns them as complex with a zero imaginary part
        self.cooling_rates, self.temperature_modes = (part.real for part in np.linalg.eig(temperature_rates))
        self.inverse_temperature_modes = np.linalg.inv(self.temperature_modes)
        self.heat_modes = self.inverse_temperature_modes @ np.array([
            1 / c_sensor_j_per_c,
            0.0,
        ])
        self.time_step_s = None
        self.reset(25.0)

    # resets both nodes to the pack's starting temperature
    def reset(self, initial_temp_C):
        if not math.isfinite(initial_temp_C):
            raise ValueError("Initial temperature must be finite")
        self.sensor_temp_C = float(initial_temp_C)
        self.pack_temp_C = float(initial_temp_C)

    # pack current is positive for discharge; regen still makes heat
    def getHeat_W(self, current_A):
        if not math.isfinite(current_A):
            raise ValueError("Current must be finite")
        return (
            self.q_i2_coeff * current_A**2
            + self.q_iabs_coeff * abs(current_A)
            + self.q_bias_w
        )

    # reuse these factors while the time step stays the same
    def updateTimeStep(self, dt_s):
        self.cooling_factors = np.exp(self.cooling_rates * dt_s)
        self.heating_factors = (
            np.expm1(self.cooling_rates * dt_s)
            / self.cooling_rates
            * self.heat_modes
        )
        self.time_step_s = dt_s

    # gets the next sensor and pack temperatures without changing either state
    def getTemperaturesAfterStep_C(self, heat_W, dt_s, sink_temp_C):
        if not all(math.isfinite(value) for value in (heat_W, dt_s, sink_temp_C)):
            raise ValueError("Heat, time step and sink temperature must be finite")
        if dt_s <= 0:
            raise ValueError("Time step must be positive")
        if self.time_step_s != dt_s:
            self.updateTimeStep(dt_s)

        temperatures_above_sink_C = np.array([
            self.sensor_temp_C - sink_temp_C,
            self.pack_temp_C - sink_temp_C,
        ])
        old_modes = self.inverse_temperature_modes @ temperatures_above_sink_C
        next_modes = (
            self.cooling_factors * old_modes
            + self.heating_factors * heat_W
        )
        next_temperatures_C = self.temperature_modes @ next_modes + sink_temp_C
        return float(next_temperatures_C[0]), float(next_temperatures_C[1])

    # runs one thermal step and carries both temperatures into the next step
    def update(self, heat_W, dt_s, sink_temp_C):
        self.sensor_temp_C, self.pack_temp_C = self.getTemperaturesAfterStep_C(
            heat_W,
            dt_s,
            sink_temp_C,
        )
        return self.sensor_temp_C


class OneNodeThermalModel:
    # one temperature with current-dependent heating and a cooling resistance
    def __init__(self, c_th_j_per_c, r_th_c_per_w, q_i2_coeff,
                 q_iabs_coeff, q_bias_w, regen_heat_multiplier=1.0):
        if any(not math.isfinite(value) or value <= 0
               for value in (c_th_j_per_c, r_th_c_per_w)):
            raise ValueError("Heat capacity and resistance must be finite and positive")
        if any(not math.isfinite(value) or value < 0
               for value in (q_i2_coeff, q_iabs_coeff, q_bias_w, regen_heat_multiplier)):
            raise ValueError("Heat coefficients must be finite and nonnegative")
        self.c_th_j_per_c = c_th_j_per_c
        self.r_th_c_per_w = r_th_c_per_w
        self.q_i2_coeff = q_i2_coeff
        self.q_iabs_coeff = q_iabs_coeff
        self.q_bias_w = q_bias_w
        self.regen_heat_multiplier = regen_heat_multiplier
        self.temperature_label = "SR16 hottest-cell temperature proxy"
        self.reset(25.0)

    # starts at the requested pack temperature
    def reset(self, initial_temp_C):
        if not math.isfinite(initial_temp_C):
            raise ValueError("Initial temperature must be finite")
        self.sensor_temp_C = float(initial_temp_C)
        self.pack_temp_C = float(initial_temp_C)

    # current-dependent heat plus constant bias
    def getHeat_W(self, current_A):
        if not math.isfinite(current_A):
            raise ValueError("Current must be finite")
        heat_W = (
            self.q_i2_coeff * current_A**2
            + self.q_iabs_coeff * abs(current_A)
            + self.q_bias_w
        )
        if current_A < 0:
            heat_W *= self.regen_heat_multiplier
        return max(heat_W, 0.0)

    # exact constant-heat step; does not apply measurement correction
    def getTemperaturesAfterStep_C(self, heat_W, dt_s, sink_temp_C):
        if not all(math.isfinite(value) for value in (heat_W, dt_s, sink_temp_C)):
            raise ValueError("Heat, time step and sink temperature must be finite")
        if dt_s <= 0:
            raise ValueError("Time step must be positive")
        steady_temp_C = sink_temp_C + heat_W * self.r_th_c_per_w
        time_constant_s = self.c_th_j_per_c * self.r_th_c_per_w
        temperature_change_C = (steady_temp_C - self.sensor_temp_C) * (
            -math.expm1(-dt_s / time_constant_s)
        )
        next_temp_C = self.sensor_temp_C + temperature_change_C
        return next_temp_C, next_temp_C

    # both outputs refer to the same node in this model
    def update(self, heat_W, dt_s, sink_temp_C):
        self.sensor_temp_C, self.pack_temp_C = self.getTemperaturesAfterStep_C(
            heat_W, dt_s, sink_temp_C,
        )
        return self.sensor_temp_C




class LPTNBatteryPack(TractiveBatteryPack):
    # same electrical pack, with a different temperature model
    def __init__(self, *, thermal_model, **pack_settings):
        self.thermal_model = deepcopy(thermal_model)
        super().__init__(**pack_settings)

    @classmethod
    def fromPack(cls, pack, thermal_model):
        settings = {
            name: deepcopy(getattr(pack, name))
            for name in (
                "mass_kg", "cell", "series_cells", "parallel_cells",
                "specific_heat_jpkgk", "thermal_resistance_kpw", "num_modules",
                "cells_per_module", "thermistors_per_module",
                "cell_voltage_offsets_mV", "temperature_offsets_C",
                "hv_sense_offset_V", "initial_soc", "initial_temp_C", "ambient_temp_C",
            )
        }
        return cls(thermal_model=thermal_model, **settings)

    def reset(self, initial_soc=None, initial_temp_C=None, ambient_temp_C=None):
        super().reset(initial_soc, initial_temp_C, ambient_temp_C)
        self.thermal_model.reset(self.temp_C)

    def getPackHeat_W(self, pack_current_A):
        return self.thermal_model.getHeat_W(pack_current_A)

    # preview the step without changing the thermal state
    def getTemperatureAfterStep_C(self, heat_W, dt_s):
        return self.thermal_model.getTemperaturesAfterStep_C(
            heat_W, dt_s, self.ambient_temp_C,
        )[0]

    def update(self, current_A, dt_s):
        super().update(current_A, dt_s)
        self.thermal_model.update(self.heat_W, dt_s, self.ambient_temp_C)

    def advanceConstantCurrent(self, current_A, duration_s):
        super().advanceConstantCurrent(current_A, duration_s)
        self.thermal_model.update(self.heat_W, duration_s, self.ambient_temp_C)

    @property
    def pack_node_temp_C(self):
        return self.thermal_model.pack_temp_C
