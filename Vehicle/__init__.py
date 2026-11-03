"""Public model constructors. Physical implementations live in the subpackages."""


def create_sr18():
    """Fresh four-motor electrical car, connected to the longitudinal chassis."""
    from Vehicle.MainConfigs.SR18 import create_sr18 as create
    return create()


def create_skidpad():
    """Fresh planar car with four wheel-torque inputs and independent wheel states."""
    from Vehicle.MainConfigs.SR18_skidpad import create_skidpad as create
    return create()


__all__ = ["create_sr18", "create_skidpad"]
