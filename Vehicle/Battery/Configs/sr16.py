from Vehicle.Battery.LPTNThermalModel import OneNodeThermalModel


# sr16 thermal model
def create_sr16_thermal_model():
    return OneNodeThermalModel(
        c_th_j_per_c=132044.93125480562,
        r_th_c_per_w=2.7522758451807494,
        q_i2_coeff=0.0011434472377465895,
        q_iabs_coeff=9.315271165615918,
        q_bias_w=672.4181820818349,
        regen_heat_multiplier=1.05,
    )


from Vehicle.Battery.TractiveBatteryPack import TractiveBatteryPack


# sr16 battery pack
def create_sr16_battery(*, mass_kg, cell, initial_soc=1.0,
                        initial_temp_C=25.0, ambient_temp_C=25.0, thermal_model="lptn"):
    battery = TractiveBatteryPack(
        mass_kg=mass_kg,
        cell=cell,
        series_cells=96,
        parallel_cells=8,
        num_modules=8,
        cells_per_module=12,
        initial_soc=initial_soc,
        initial_temp_C=initial_temp_C,
        ambient_temp_C=ambient_temp_C,
    )

    if thermal_model == "legacy":
        return battery
    if thermal_model != "lptn":
        raise ValueError("thermal_model must be lptn or legacy")
    from Vehicle.Battery.LPTNThermalModel import LPTNBatteryPack
    return LPTNBatteryPack.fromPack(battery, create_sr16_thermal_model())
