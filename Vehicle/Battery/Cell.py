import math


def estimateOpenCircuitVoltage_V(
    loaded_voltage_V,
    current_A,
    internal_resistance_ohm,
):
    voltage_drop_V = current_A * internal_resistance_ohm
    return loaded_voltage_V + voltage_drop_V


class Cell:
    def __init__(
        self,
        capacity_ah=5.0,
        nominal_voltage_v=3.6,
        internal_resistance_ohm=6.7e-3,
        max_discharge_current_a=125.0,
        ocv_soc=None,
        ocv_voltage_v=None,
    ):
        self.capacity_ah = capacity_ah

        self.nominal_voltage_v = nominal_voltage_v

        self.internal_resistance_ohm = internal_resistance_ohm

        self.max_discharge_current_a = max_discharge_current_a

        if ocv_soc is None:
            ocv_soc = [0.00, 0.20, 0.50, 0.80, 1.00]
        if ocv_voltage_v is None:
            ocv_voltage_v = [2.50, 3.55, 3.70, 3.88, 4.20]

        if len(ocv_soc) != len(ocv_voltage_v):
            raise ValueError("SOC and voltage curves must have the same length")
        if len(ocv_soc) < 2:
            raise ValueError("The cell voltage curve needs at least two points")
        if any(ocv_soc[i] >= ocv_soc[i + 1] for i in range(len(ocv_soc) - 1)):
            raise ValueError("SOC curve values must be in increasing order")

        self.ocv_soc = ocv_soc
        self.ocv_voltage_v = ocv_voltage_v
        self.min_voltage_v = ocv_voltage_v[0]
        self.max_voltage_v = ocv_voltage_v[-1]

    def getOpenCircuitVoltage_V(self, soc):
        if soc < 0 or soc > 1:
            raise ValueError("SOC must be between 0 and 1")

        for i in range(len(self.ocv_soc) - 1):
            if soc <= self.ocv_soc[i + 1]:
                return self.interpolateVoltage(
                    soc,
                    self.ocv_soc[i],
                    self.ocv_soc[i + 1],
                    self.ocv_voltage_v[i],
                    self.ocv_voltage_v[i + 1],
                )
        return self.ocv_voltage_v[-1]

    def interpolateVoltage(
        self,
        soc,
        lower_soc,
        upper_soc,
        lower_voltage_V,
        upper_voltage_V,
    ):
        amount_between_points = (soc - lower_soc) / (upper_soc - lower_soc)
        voltage_difference_V = upper_voltage_V - lower_voltage_V
        return lower_voltage_V + amount_between_points * voltage_difference_V

    def getVoltageDrop_V(self, current_A):
        return current_A * self.internal_resistance_ohm

    def getTerminalVoltage_V(self, soc, current_A):
        open_circuit_voltage_V = self.getOpenCircuitVoltage_V(soc)
        return open_circuit_voltage_V - self.getVoltageDrop_V(current_A)

    def getHeat_W(self, current_A):
        return current_A**2 * self.internal_resistance_ohm

    def getUsedCapacity_Ah(self, current_A, dt):
        seconds_per_hour = 3600
        return current_A * dt / seconds_per_hour

    def getNextSoc(self, soc, current_A, dt):
        used_capacity_Ah = self.getUsedCapacity_Ah(current_A, dt)
        return soc - used_capacity_Ah / self.capacity_ah

    def getCurrentForPower_A(self, power_W, open_circuit_voltage_V):
        discriminant = (
            open_circuit_voltage_V**2
            - 4 * self.internal_resistance_ohm * power_W
        )
        if discriminant < 0:
            raise ValueError("Cell cannot supply that much power")

        return 2 * power_W / (
            open_circuit_voltage_V + math.sqrt(discriminant)
        )

    def getMaxDischargeCurrent_A(self, soc):
        open_circuit_voltage_V = self.getOpenCircuitVoltage_V(soc)
        voltage_limited_current_A = (
            (open_circuit_voltage_V - self.min_voltage_v)
            / self.internal_resistance_ohm
        )
        return min(
            self.max_discharge_current_a,
            max(0.0, voltage_limited_current_A),
        )
