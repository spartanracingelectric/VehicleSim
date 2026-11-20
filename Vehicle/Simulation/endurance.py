from dataclasses import dataclass

# A long run of just the battery and the BMS: the power the driver asks for over time, with a power limit on top.
# The full car runs about 3x slower than real time and pack temperature only needs pack current, so this skips the drives.
# The demand keeps its length: on track a lower limit would make the same distance take longer.

@dataclass
class EnduranceRun:
    dt_s: float                     # 0.1 s or shorter: heat goes with current squared, so longer steps lose the peaks
    demand_power_W: object          # battery power asked for at each step (W). negative = regen
    start_soc: float = 1.0
    start_temp_C: float = 25.0
    battery: object = None          # None = the default sr17 pack
