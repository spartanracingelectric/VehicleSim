def getTireLoads_N(car, u_mps, ax_mps2, ay_mps2):
    Weight = car.getWeight()
    front_load = Weight * (1 - car.rear_weight_distribution) + car.aero.getFrontDownforce_N(u_mps) - car.mass * ax_mps2 * car.cg_height / car.wheelbase
    rear_load = Weight * car.rear_weight_distribution + car.aero.getRearDownforce_N(u_mps) + car.mass * ax_mps2 * car.cg_height / car.wheelbase

    front_cornering_transfer_between_wheels= car.front_lateral_transfer * car.mass * ay_mps2 * car.cg_height / car.front_track
    rear_cornering_transfer_between_wheels= (1-car.front_lateral_transfer)* car.mass * ay_mps2 * car.cg_height / car.rear_track

    FL = max(0.0,front_load/2-front_cornering_transfer_between_wheels)
    FR = max(0.0, front_load/2+front_cornering_transfer_between_wheels)
    RL = max(0.0, rear_load/2-rear_cornering_transfer_between_wheels)
    RR = max(0.0, rear_load/2+rear_cornering_transfer_between_wheels)


    return FL, FR, RL, RR
