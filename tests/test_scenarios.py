import pytest
from Vehicle.Simulation.scenarios import SCENARIOS, runAndCheck


@pytest.mark.parametrize("scenario", SCENARIOS, ids=lambda s: s.name)
def test_straightline_scenario(scenario):
    runAndCheck(scenario)
