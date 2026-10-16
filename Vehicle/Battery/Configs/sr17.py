from Vehicle.Battery.Cell import Cell
from Vehicle.Battery.TractiveBatteryPack import TractiveBatteryPack


def estimateOpenCircuitVoltage_V(
    loaded_voltage_V,
    current_A,
    internal_resistance_ohm,
):
    voltage_drop_V = current_A * internal_resistance_ohm
    return loaded_voltage_V + voltage_drop_V

# jp50p1 5a discharge test
test_current_A = 5.0
cell_resistance_ohm = 6.7e-3

# pack layout
num_modules = 10
cells_per_module = 14
thermistors_per_module = 10
series_cells = num_modules * cells_per_module
parallel_cells = 3

# thermal model
specific_heat_jpkgk = 900.0      # joules per kilogram kelvin
thermal_resistance_kpw = None    # kelvin per watt

# sensor offsets
cell_voltage_offsets_mV = [0.0] * series_cells
temperature_offsets_C = [0.0] * (
    num_modules * thermistors_per_module
)
hv_sense_offset_V = 0.0

# starting state
initial_soc = 1.0
initial_temp_C = 25.0
ambient_temp_C = 25.0

test_data = [
    (0.000, 2.810),
    (0.008, 2.840),
    (0.017, 2.870),
    (0.027, 2.900),
    (0.036, 2.930),
    (0.045, 2.950),
    (0.055, 2.970),
    (0.064, 3.000),
    (0.074, 3.020),
    (0.084, 3.040),
    (0.093, 3.060),
    (0.103, 3.080),
    (0.113, 3.100),
    (0.123, 3.120),
    (0.133, 3.140),
    (0.143, 3.160),
    (0.154, 3.190),
    (0.165, 3.200),
    (0.175, 3.220),
    (0.185, 3.240),
    (0.195, 3.260),
    (0.206, 3.280),
    (0.216, 3.300),
    (0.227, 3.310),
    (0.238, 3.330),
    (0.248, 3.340),
    (0.259, 3.360),
    (0.270, 3.370),
    (0.281, 3.390),
    (0.292, 3.400),
    (0.303, 3.420),
    (0.314, 3.430),
    (0.325, 3.450),
    (0.336, 3.470),
    (0.347, 3.480),
    (0.358, 3.500),
    (0.369, 3.510),
    (0.380, 3.530),
    (0.392, 3.540),
    (0.403, 3.560),
    (0.415, 3.580),
    (0.426, 3.590),
    (0.438, 3.600),
    (0.449, 3.620),
    (0.461, 3.630),
    (0.472, 3.640),
    (0.484, 3.660),
    (0.496, 3.670),
    (0.508, 3.680),
    (0.519, 3.690),
    (0.531, 3.700),
    (0.543, 3.710),
    (0.555, 3.720),
    (0.567, 3.730),
    (0.579, 3.740),
    (0.591, 3.750),
    (0.603, 3.760),
    (0.615, 3.770),
    (0.627, 3.770),
    (0.639, 3.780),
    (0.652, 3.790),
    (0.664, 3.800),
    (0.676, 3.820),
    (0.688, 3.830),
    (0.700, 3.840),
    (0.713, 3.860),
    (0.725, 3.870),
    (0.738, 3.890),
    (0.750, 3.900),
    (0.763, 3.910),
    (0.775, 3.920),
    (0.788, 3.920),
    (0.800, 3.930),
    (0.813, 3.940),
    (0.826, 3.940),
    (0.838, 3.950),
    (0.851, 3.950),
    (0.864, 3.960),
    (0.876, 3.960),
    (0.889, 3.970),
    (0.902, 3.970),
    (0.915, 3.970),
    (0.927, 3.980),
    (0.940, 3.980),
    (0.953, 3.990),
    (0.966, 4.000),
    (0.978, 4.000),
    (0.991, 4.020),
    (0.997, 4.040),
    (1.000, 4.040),
]

# turn the 5a test voltages into no-load voltages
jp50p1 = Cell(
    capacity_ah=5.0,
    nominal_voltage_v=3.6,
    internal_resistance_ohm=cell_resistance_ohm,
    max_discharge_current_a=125.0,
    ocv_soc=[row[0] for row in test_data],
    ocv_voltage_v=[
        estimateOpenCircuitVoltage_V(
            row[1],
            test_current_A,
            cell_resistance_ohm,
        )
        for row in test_data
    ],
)

# sr17 battery pack
sr17 = TractiveBatteryPack(
    mass_kg=50.8,
    cell=jp50p1,
    series_cells=series_cells,
    parallel_cells=parallel_cells,
    specific_heat_jpkgk=specific_heat_jpkgk,
    thermal_resistance_kpw=thermal_resistance_kpw,
    num_modules=num_modules,
    cells_per_module=cells_per_module,
    thermistors_per_module=thermistors_per_module,
    cell_voltage_offsets_mV=cell_voltage_offsets_mV,
    temperature_offsets_C=temperature_offsets_C,
    hv_sense_offset_V=hv_sense_offset_V,
    initial_soc=initial_soc,
    initial_temp_C=initial_temp_C,
    ambient_temp_C=ambient_temp_C,
)
