"""Compatibility name for older scripts; use vesc_srinverter in new configurations."""
from copy import deepcopy
from Vehicle.Inverter.Configs.vesc_srinverter import vesc_srinverter

inverter1 = deepcopy(vesc_srinverter)
