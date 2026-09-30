import numpy as np


# a flat map still needs two breakpoints per axis so the 2d interpolation
# has something to interpolate between, so spread them wide enough that
# nothing the sim does ever lands outside
FLAT_SOC_BREAKPOINTS = [0.0, 1.0]
FLAT_TEMPERATURE_BREAKPOINTS_C = [-20.0, 60.0]


class ECMParameterMap:
    """Cell level R0, R1 and tau lookup tables over soc and temperature.

    battery_model.md level 3 asks for lookup tables for R0, Rj and
    tau_j = Rj * Cj over soc and temperature, instead of single numbers.

    Every grid is indexed [temperature, soc], so grid[i][j] is the value at
    temperature_breakpoints_C[i] and soc_breakpoints[j]. That matches the
    order pybamm wants for a 2d interpolant.

    Values are clamped at the edges of the table, never extrapolated,
    because extrapolating a resistance fit can go negative.
    """

    def __init__(
        self,
        soc_breakpoints,
        temperature_breakpoints_C,
        r0_ohm,
        r1_ohm,
        tau_s,
    ):
        self.soc_breakpoints = np.asarray(soc_breakpoints, dtype=float)
        self.temperature_breakpoints_C = np.asarray(
            temperature_breakpoints_C,
            dtype=float,
        )

        self.checkBreakpoints(self.soc_breakpoints, "SOC")
        self.checkBreakpoints(self.temperature_breakpoints_C, "Temperature")

        self.r0_ohm = self.checkGrid(r0_ohm, "R0")
        self.r1_ohm = self.checkGrid(r1_ohm, "R1")
        self.tau_s = self.checkGrid(tau_s, "Tau")

    # a map made from single numbers, the same value everywhere
    # this is what the model uses until an hppc pulse test fills in real tables
    @classmethod
    def fromConstants(cls, r0_ohm, r1_ohm, tau_s):
        shape = (
            len(FLAT_TEMPERATURE_BREAKPOINTS_C),
            len(FLAT_SOC_BREAKPOINTS),
        )
        return cls(
            soc_breakpoints=FLAT_SOC_BREAKPOINTS,
            temperature_breakpoints_C=FLAT_TEMPERATURE_BREAKPOINTS_C,
            r0_ohm=np.full(shape, float(r0_ohm)),
            r1_ohm=np.full(shape, float(r1_ohm)),
            tau_s=np.full(shape, float(tau_s)),
        )

    # checks one axis of the table
    def checkBreakpoints(self, breakpoints, name):
        if len(breakpoints) < 2:
            raise ValueError(f"{name} breakpoints need at least two points")
        if any(
            breakpoints[i] >= breakpoints[i + 1]
            for i in range(len(breakpoints) - 1)
        ):
            raise ValueError(f"{name} breakpoints must be in increasing order")

    # checks one grid against the two axes
    def checkGrid(self, grid, name):
        grid = np.asarray(grid, dtype=float)
        expected_shape = (
            len(self.temperature_breakpoints_C),
            len(self.soc_breakpoints),
        )
        if grid.shape != expected_shape:
            raise ValueError(
                f"{name} grid must be shaped "
                f"[temperature, soc] {expected_shape}, got {grid.shape}"
            )
        if np.any(grid <= 0):
            raise ValueError(f"{name} values must be positive")
        return grid

    # gets a value out of one grid, bilinear with the edges clamped
    def lookup(self, grid, soc, temperature_C):
        # np.interp clamps on its own outside the breakpoints
        value_at_soc = np.array([
            np.interp(soc, self.soc_breakpoints, row) for row in grid
        ])
        return float(
            np.interp(
                temperature_C,
                self.temperature_breakpoints_C,
                value_at_soc,
            )
        )

    # gets cell series resistance
    def getR0_Ohm(self, soc, temperature_C):
        return self.lookup(self.r0_ohm, soc, temperature_C)

    # gets cell polarization resistance
    def getR1_Ohm(self, soc, temperature_C):
        return self.lookup(self.r1_ohm, soc, temperature_C)

    # gets cell rc time constant
    def getTau_s(self, soc, temperature_C):
        return self.lookup(self.tau_s, soc, temperature_C)

    # gets cell polarization capacitance, tau / r1
    def getC1_F(self, soc, temperature_C):
        return self.getTau_s(soc, temperature_C) / self.getR1_Ohm(
            soc,
            temperature_C,
        )

    # true when every grid holds a single repeated value
    def isFlat(self):
        return all(
            np.allclose(grid, grid.flat[0])
            for grid in (self.r0_ohm, self.r1_ohm, self.tau_s)
        )

    # gets the smallest and largest value in one grid, for printing
    def getRange(self, grid):
        return float(np.min(grid)), float(np.max(grid))
