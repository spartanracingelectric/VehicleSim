import numpy as np
import pytest

from Vehicle.Chassis.QuarterCarSuspension import QuarterCarSuspension
from Vehicle.Chassis.ChassisConfig import ChassisConfig
from Vehicle.Tire.TireConfig import TireConfig
from Vehicle.Suspension import DesignCase, simulate


def test_quartercar_simulation_returns_dynamic_corner_outputs():
    result = simulate(DesignCase())

    assert len(result.time) > 1
    assert result.sprung_acceleration.shape == result.time.shape
    assert result.contact_force.shape == result.time.shape
    assert np.max(result.contact_force) > 0


def test_four_corner_bridge_starts_at_static_load_and_advances():
    cases = [DesignCase() for _ in range(4)]
    suspension = QuarterCarSuspension(cases)
    initial_loads = list(suspension.contact_forces_N)

    loads = suspension.step(1e-4)

    assert loads == pytest.approx(initial_loads, rel=1e-4)
    assert suspension.time_s == pytest.approx(1e-4)


def test_quartercar_bridge_requires_four_cases():
    with pytest.raises(ValueError, match="four cases"):
        QuarterCarSuspension([DesignCase()])


def test_chassis_passes_one_authoritative_timestep_to_suspension():
    class RecordingSuspension:
        def __init__(self):
            self.steps = []

        def step(self, dt_s):
            self.steps.append(dt_s)
            return [100.0] * 4

    suspension = RecordingSuspension()
    chassis = ChassisConfig(
        0.0, 0.0, 1.5, 0.3, 0.5, 0.2,
        suspension_model=suspension,
    )
    tire = TireConfig(radius_m=0.2)

    chassis.update([0.0] * 4, [0.0] * 4, [0.1] * 4, 100.0, tire, 0.001)
    chassis.update([0.0] * 4, [0.0] * 4, [0.1] * 4, 100.0, tire, 0.001)

    assert suspension.steps == [0.001, 0.001]
    assert chassis.getTireLoads_N() == [100.0] * 4
