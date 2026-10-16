from Vehicle.Brake.BrakeConfig import BrakeConfig

# TODO: every number here is a placeholder (typical FSAE hardware), replace with SR18's real brake parts
# picked so the fronts lock before the rears (~50% vs ~65% pedal on tire1), rears locking first spins the car
brake1 = BrakeConfig(
    mass_kg=10, #placeholder
    max_pedal_force_N=500.0,                  # driver's push at 100% pedal
    pedal_ratio=5.0,                          # pedal leverage
    front_bias=0.65,                          # balance bar, 65% of the push to the front master cylinder
    front_master_cylinder_bore_m=0.015875,    # 5/8"
    rear_master_cylinder_bore_m=0.015875,     # 5/8"
    front_caliper_piston_diameter_m=0.025,
    front_caliper_pistons_per_side=2,         # 4 piston opposed caliper
    rear_caliper_piston_diameter_m=0.022,
    rear_caliper_pistons_per_side=1,          # 2 piston opposed caliper
    pad_mu=0.45,
    front_rotor_radius_m=0.085,               # effective radius (middle of the pad)
    rear_rotor_radius_m=0.080,
)
