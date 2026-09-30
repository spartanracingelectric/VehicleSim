import numpy as np
import pybamm

from Vehicle.Battery.ECMParameterMap import ECMParameterMap


# rc branch values for one cell
# these are placeholders
# an hppc pulse test is needed to fit them (build-up step 7 in battery_model.md)
PLACEHOLDER_CELL_R1_OHM = 2.0e-3
PLACEHOLDER_CELL_TAU_S = 30.0

# pybamm has a maximum and minimum soc event, so soc cannot start exactly at the ends
SOC_MARGIN = 1e-3


class TheveninECM:
    """Level 3 dynamic ecm from battery_model.md using pybamm

    Pack power and current are positive when discharging.

    The pack is given to pybamm as one big equivalent cell so that pybamm
    reports pack voltage directly:

        ocv_pack     = series_cells * ocv_cell
        capacity     = parallel_cells * cell capacity
        r0_pack      = series_cells / parallel_cells * r0_cell + bus resistance
        r1_pack      = series_cells / parallel_cells * r1_cell
        c1_pack      = parallel_cells / series_cells * c1_cell

    c1 scales the other way so the time constant r1 * c1 is unchanged.

    R0, R1 and tau come from an ECMParameterMap, the soc and temperature
    lookup tables battery_model.md level 3 asks for. Passing no map builds
    a flat one from the single cell values, which is the same behaviour as
    before the tables existed.

    pybamm covers steps 6 to 9 of the battery timestep order.
    Steps 1 to 5 and 10 stay here because pybamm has no power limit model.
    """

    def __init__(
        self,
        pack,
        cell_r1_ohm=PLACEHOLDER_CELL_R1_OHM,
        cell_tau_s=PLACEHOLDER_CELL_TAU_S,
        bus_resistance_ohm=0.0,
        entropic_change_vpk=0.0,
        thermal_resistance_kpw=None,
        parameter_map=None,
    ):
        if cell_r1_ohm <= 0 or cell_tau_s <= 0:
            raise ValueError("RC branch values must be positive")

        self.pack = pack
        self.cell = pack.cell
        self.series_cells = pack.series_cells
        self.parallel_cells = pack.parallel_cells

        self.cell_r1_ohm = cell_r1_ohm
        self.cell_tau_s = cell_tau_s
        self.bus_resistance_ohm = bus_resistance_ohm

        # no dU/dT data, so reversible heat is off by default
        self.entropic_change_vpk = entropic_change_vpk

        # falls back to the pack value, None means adiabatic
        if thermal_resistance_kpw is None:
            thermal_resistance_kpw = pack.thermal_resistance_kpw
        self.thermal_resistance_kpw = thermal_resistance_kpw

        self.is_placeholder_rc = (
            parameter_map is None
            and cell_r1_ohm == PLACEHOLDER_CELL_R1_OHM
            and cell_tau_s == PLACEHOLDER_CELL_TAU_S
        )

        # no map given, so build a flat one out of the single cell values
        if parameter_map is None:
            parameter_map = ECMParameterMap.fromConstants(
                r0_ohm=self.cell.internal_resistance_ohm,
                r1_ohm=cell_r1_ohm,
                tau_s=cell_tau_s,
            )
        self.cell_map = parameter_map
        self.pack_map = self.buildPackMap()

        self.ocv_soc = np.array(self.cell.ocv_soc)
        self.ocv_cell_voltage_v = np.array(self.cell.ocv_voltage_v)

        self.model = self.buildModel()
        self.parameter_values = self.buildParameterValues()
        self.simulation = None
        self.reset()

    # pack values, from the pack resistance section of battery_model.md
    # turns the cell tables into pack tables on the same breakpoints
    def buildPackMap(self):
        series_over_parallel = self.series_cells / self.parallel_cells
        return ECMParameterMap(
            soc_breakpoints=self.cell_map.soc_breakpoints,
            temperature_breakpoints_C=self.cell_map.temperature_breakpoints_C,
            r0_ohm=(
                series_over_parallel * self.cell_map.r0_ohm
                + self.bus_resistance_ohm
            ),
            r1_ohm=series_over_parallel * self.cell_map.r1_ohm,
            # tau is a time constant, so it survives the pack scaling
            # unchanged and c1 follows from tau / r1_pack
            tau_s=self.cell_map.tau_s,
        )

    # gets the soc and temperature the tables are read at
    def getMapState(self, soc=None, temperature_C=None):
        if soc is None:
            soc = self.soc
        if temperature_C is None:
            temperature_C = self.temp_C
        return soc, temperature_C

    # gets pack series resistance
    def getPackR0_Ohm(self, soc=None, temperature_C=None):
        soc, temperature_C = self.getMapState(soc, temperature_C)
        return self.pack_map.getR0_Ohm(soc, temperature_C)

    # gets pack polarization resistance
    def getPackR1_Ohm(self, soc=None, temperature_C=None):
        soc, temperature_C = self.getMapState(soc, temperature_C)
        return self.pack_map.getR1_Ohm(soc, temperature_C)

    # gets pack polarization capacitance, chosen so the time constant is kept
    def getPackC1_F(self, soc=None, temperature_C=None):
        soc, temperature_C = self.getMapState(soc, temperature_C)
        return self.pack_map.getC1_F(soc, temperature_C)

    # gets the rc time constant
    def getTau_s(self, soc=None, temperature_C=None):
        soc, temperature_C = self.getMapState(soc, temperature_C)
        return self.pack_map.getTau_s(soc, temperature_C)

    # gets pack capacity seen by the soc integrator
    def getPackCapacity_Ah(self):
        return self.parallel_cells * self.cell.capacity_ah

    # gets pack voltage with no current flowing
    def getPackOpenCircuitVoltage_V(self, soc):
        cell_voltage_V = self.cell.getOpenCircuitVoltage_V(soc)
        return cell_voltage_V * self.series_cells

    # builds the pybamm model
    def buildModel(self):
        return pybamm.equivalent_circuit.Thevenin(
            options={
                "number of rc elements": 1,
                "operating mode": "power",
            }
        )

    # builds one pybamm lookup over the pack tables
    # pybamm calls these with temperature in degC, current in amps and
    # soc from 0 to 1, and only temperature and soc are used
    def buildLookup(self, grid, name):
        soc_breakpoints = self.pack_map.soc_breakpoints
        temperature_breakpoints_C = self.pack_map.temperature_breakpoints_C

        def lookup(T_cell, current, soc):
            # clamp at the table edges, extrapolating a resistance fit
            # can hand back a negative number
            clamped_temperature_C = pybamm.minimum(
                pybamm.maximum(T_cell, temperature_breakpoints_C[0]),
                temperature_breakpoints_C[-1],
            )
            clamped_soc = pybamm.minimum(
                pybamm.maximum(soc, soc_breakpoints[0]),
                soc_breakpoints[-1],
            )
            return pybamm.Interpolant(
                [temperature_breakpoints_C, soc_breakpoints],
                grid,
                [clamped_temperature_C, clamped_soc],
                name=name,
            )

        return lookup

    # builds the pybamm parameters for the whole pack
    def buildParameterValues(self):
        pack_ocv_voltage_v = self.ocv_cell_voltage_v * self.series_cells
        ocv_soc = self.ocv_soc

        if self.thermal_resistance_kpw is None:
            cell_to_jig_wpk = 0.0
        else:
            cell_to_jig_wpk = 1.0 / self.thermal_resistance_kpw

        parameter_values = pybamm.ParameterValues("ECM_Example")
        parameter_values.update(
            {
                "Open-circuit voltage [V]": lambda soc: pybamm.Interpolant(
                    ocv_soc,
                    pack_ocv_voltage_v,
                    soc,
                    name="pack ocv",
                ),
                "Cell capacity [A.h]": self.getPackCapacity_Ah(),
                "Nominal cell capacity [A.h]": self.getPackCapacity_Ah(),
                "Upper voltage cut-off [V]": (
                    self.cell.max_voltage_v * self.series_cells
                ),
                "Lower voltage cut-off [V]": (
                    self.cell.min_voltage_v * self.series_cells
                ),
                "R0 [Ohm]": self.buildLookup(
                    self.pack_map.r0_ohm,
                    "pack r0",
                ),
                "R1 [Ohm]": self.buildLookup(
                    self.pack_map.r1_ohm,
                    "pack r1",
                ),
                "C1 [F]": self.buildLookup(
                    self.pack_map.tau_s / self.pack_map.r1_ohm,
                    "pack c1",
                ),
                "Element-1 initial overpotential [V]": 0.0,
                "Entropic change [V/K]": self.entropic_change_vpk,
                "Power function [W]": pybamm.InputParameter("power"),
                "RCR lookup limit [A]": self.pack.max_discharge_current_a,
                # the jig is pinned to ambient so the two node pybamm thermal
                # model collapses to the single Cth and Rth node in the doc
                "Cell thermal mass [J/K]": self.pack.heat_capacity_jpk,
                "Jig thermal mass [J/K]": 1e9,
                "Cell-jig heat transfer coefficient [W/K]": cell_to_jig_wpk,
                "Jig-air heat transfer coefficient [W/K]": 1e9,
            },
            check_already_exists=False,
        )
        return parameter_values

    # resets the model state
    def reset(self, initial_soc=None, initial_temp_C=None, ambient_temp_C=None):
        if initial_soc is None:
            initial_soc = self.pack.initial_soc
        if initial_temp_C is None:
            initial_temp_C = self.pack.initial_temp_C
        if ambient_temp_C is None:
            ambient_temp_C = self.pack.ambient_temp_C

        if initial_soc < 0 or initial_soc > 1:
            raise ValueError("SOC must be between 0 and 1")

        # keep soc off the event bounds
        started_soc = min(max(initial_soc, SOC_MARGIN), 1.0 - SOC_MARGIN)

        self.parameter_values.update(
            {
                "Initial SoC": started_soc,
                "Initial temperature [K]": initial_temp_C + 273.15,
                "Ambient temperature [K]": ambient_temp_C + 273.15,
            }
        )

        self.simulation = pybamm.Simulation(
            self.model,
            parameter_values=self.parameter_values,
        )
        self.solution = None

        self.elapsed_time_s = 0.0
        self.soc = started_soc
        self.temp_C = initial_temp_C
        self.ambient_temp_C = ambient_temp_C
        self.polarization_voltage_V = 0.0
        self.open_circuit_voltage_V = self.getPackOpenCircuitVoltage_V(
            started_soc
        )
        self.terminal_voltage_V = self.open_circuit_voltage_V
        self.current_A = 0.0
        self.terminal_power_W = 0.0
        self.heat_W = 0.0
        self.discharged_energy_kWh = 0.0
        self.charged_energy_kWh = 0.0
        self.shunt_coulomb_count_C = 0.0

    # step 3 of the timestep order
    # gets the most power the pack can give out right now
    def getDischargeLimit_W(self):
        cell_current_A = self.cell.getMaxDischargeCurrent_A(self.soc)
        current_A = cell_current_A * self.parallel_cells
        voltage_V = (
            self.open_circuit_voltage_V
            - self.polarization_voltage_V
            - current_A * self.getPackR0_Ohm()
        )
        return max(0.0, voltage_V * current_A)

    # gets the most power the pack can take in right now
    def getChargeLimit_W(self):
        max_voltage_V = self.cell.max_voltage_v * self.series_cells
        headroom_V = max_voltage_V - (
            self.open_circuit_voltage_V - self.polarization_voltage_V
        )
        current_A = max(0.0, headroom_V / self.getPackR0_Ohm())
        current_A = min(current_A, self.pack.max_discharge_current_a)
        return max(0.0, max_voltage_V * current_A)

    # step 5 of the timestep order
    # clamps requested power to the limits
    def clampPower_W(self, requested_power_W):
        return min(
            max(requested_power_W, -self.getChargeLimit_W()),
            self.getDischargeLimit_W(),
        )

    # gets energy moved during one step
    def getEnergyForStep_kWh(self, power_W, dt_s):
        joules_per_kilowatt_hour = 3_600_000
        return power_W * dt_s / joules_per_kilowatt_hour

    # runs one timestep in the order given by battery_model.md
    def step(self, requested_power_W, dt_s):
        if dt_s <= 0:
            raise ValueError("Time step must be positive")

        # steps 1 to 5, soc, temperature, limits, then clamp
        power_W = self.clampPower_W(requested_power_W)

        # steps 6 to 9 are solved by pybamm
        self.solution = self.simulation.step(
            dt_s,
            inputs={"power": power_W},
            starting_solution=self.solution,
        )

        def last(name):
            return float(self.solution[name].entries[-1])

        self.current_A = last("Current [A]")
        self.terminal_voltage_V = last("Voltage [V]")
        self.open_circuit_voltage_V = last("Open-circuit voltage [V]")
        self.soc = last("SoC")
        self.temp_C = last("Cell temperature [degC]")
        self.heat_W = last("Total heat generation [W]")

        # pybamm adds its overpotentials, the doc subtracts them
        self.polarization_voltage_V = -last("Element-1 overpotential [V]")

        self.terminal_power_W = self.terminal_voltage_V * self.current_A
        energy_kWh = self.getEnergyForStep_kWh(self.terminal_power_W, dt_s)
        self.discharged_energy_kWh += max(energy_kWh, 0)
        self.charged_energy_kWh += max(-energy_kWh, 0)
        self.shunt_coulomb_count_C += self.current_A * dt_s
        self.elapsed_time_s += dt_s

        # step 10, publish
        return {
            "time_s": self.elapsed_time_s,
            "requested_power_W": requested_power_W,
            "power_W": power_W,
            "current_A": self.current_A,
            "open_circuit_voltage_V": self.open_circuit_voltage_V,
            "polarization_voltage_V": self.polarization_voltage_V,
            "terminal_voltage_V": self.terminal_voltage_V,
            "terminal_power_W": self.terminal_power_W,
            "heat_W": self.heat_W,
            "soc": self.soc,
            "temp_C": self.temp_C,
            "discharge_limit_W": self.getDischargeLimit_W(),
            "charge_limit_W": self.getChargeLimit_W(),
        }

    # runs a whole power trace and keeps the history
    def run(self, power_trace_W, dt_s):
        self.time_s = [0.0]
        self.requested_power_W = [0.0]
        self.current_A_history = [0.0]
        self.open_circuit_voltage_V_history = [self.open_circuit_voltage_V]
        self.terminal_voltage_V_history = [self.terminal_voltage_V]
        self.terminal_power_W_history = [0.0]
        self.heat_W_history = [0.0]
        self.soc_history = [self.soc]
        self.temp_C_history = [self.temp_C]
        self.discharged_energy_kWh_history = [0.0]
        self.charged_energy_kWh_history = [0.0]
        self.shunt_coulomb_count_C_history = [0.0]

        for power_W in power_trace_W:
            reading = self.step(power_W, dt_s)
            self.time_s.append(reading["time_s"])
            self.requested_power_W.append(reading["requested_power_W"])
            self.current_A_history.append(reading["current_A"])
            self.open_circuit_voltage_V_history.append(
                reading["open_circuit_voltage_V"]
            )
            self.terminal_voltage_V_history.append(
                reading["terminal_voltage_V"]
            )
            self.terminal_power_W_history.append(reading["terminal_power_W"])
            self.heat_W_history.append(reading["heat_W"])
            self.soc_history.append(reading["soc"])
            self.temp_C_history.append(reading["temp_C"])
            self.discharged_energy_kWh_history.append(
                self.discharged_energy_kWh
            )
            self.charged_energy_kWh_history.append(self.charged_energy_kWh)
            self.shunt_coulomb_count_C_history.append(
                self.shunt_coulomb_count_C
            )
        return self

    # prints one table value, as a range when the table is not flat
    def describeMapValue(self, grid, scale, unit):
        lowest, highest = self.pack_map.getRange(grid)
        if np.isclose(lowest, highest):
            return f"{lowest * scale:.1f} {unit}"
        return f"{lowest * scale:.1f} to {highest * scale:.1f} {unit}"

    # prints the model setup
    def summary(self):
        pack_c1_grid_f = self.pack_map.tau_s / self.pack_map.r1_ohm
        r0_text = self.describeMapValue(self.pack_map.r0_ohm, 1000, "mOhm")
        r1_text = self.describeMapValue(self.pack_map.r1_ohm, 1000, "mOhm")
        c1_text = self.describeMapValue(pack_c1_grid_f, 1, "F")
        tau_text = self.describeMapValue(self.pack_map.tau_s, 1, "s")
        lines = [
            f"Configuration: {self.series_cells}s{self.parallel_cells}p",
            f"Pack capacity: {self.getPackCapacity_Ah():.2f} Ah",
            f"Pack R0: {r0_text}",
            f"Pack R1: {r1_text}",
            f"Pack C1: {c1_text}",
            f"Time constant: {tau_text}",
            f"Thermal mass: {self.pack.heat_capacity_jpk:.0f} J/K",
        ]
        if self.cell_map.isFlat():
            lines.append(
                "Parameter tables: flat, no soc or temperature dependence"
            )
        else:
            lines.append(
                f"Parameter tables: "
                f"{len(self.cell_map.soc_breakpoints)} soc points x "
                f"{len(self.cell_map.temperature_breakpoints_C)} "
                f"temperature points"
            )
        if self.thermal_resistance_kpw is None:
            lines.append("Cooling: none, adiabatic")
        else:
            lines.append(f"Cooling: {self.thermal_resistance_kpw:.4f} K/W")
        if self.is_placeholder_rc:
            lines.append(
                "WARNING: R1 and C1 are placeholders, "
                "they need an HPPC pulse test to be real"
            )
        return "\n".join(lines)


def main():
    from Vehicle.Battery.Configs.sr17 import sr17

    ecm = TheveninECM(sr17)
    print(ecm.summary())

    dt_s = 0.5
    # 30 s at 50 kW then 30 s of rest, to see the rc branch recover
    power_trace_W = [50_000.0] * 60 + [0.0] * 60
    ecm.run(power_trace_W, dt_s)

    print()
    print(f"Final SOC: {ecm.soc * 100:.2f}%")
    print(f"Sag at end of load: {ecm.terminal_voltage_V_history[60]:.2f} V")
    print(f"Recovered voltage: {ecm.terminal_voltage_V:.2f} V")
    print(f"Polarization left: {ecm.polarization_voltage_V:.3f} V")
    print(f"Final temperature: {ecm.temp_C:.2f} C")
    print(f"Energy discharged: {ecm.discharged_energy_kWh:.3f} kWh")


if __name__ == "__main__":
    main()
