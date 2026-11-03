from Vehicle.Simulation.dyno import (
    STEP_TEST, SATURATION_TEST, simulate, checkCurrentSteps, checkVoltageLimit,
)


def test_current_steps_and_power_balance():
    checkCurrentSteps(simulate(STEP_TEST))


def test_field_weakening_at_voltage_limit():
    checkVoltageLimit(simulate(SATURATION_TEST))
