"""Friendly names → MoTeC CSV column labels.

Add entries here as you need more channels. Keys are stable in code;
values are the exact headers from the log.
"""

CHANNELS = {
    # time
    "time": "Time",

    # motion / attitude
    "g_lat": "G Force Lat DAQ",
    "g_vert": "G Force Vert DAQ",
    "g_long": "G Force Long DAQ",
    "steering_angle": "Steering Wheel Angle",
    "yaw": "IMU Yaw",

    # wheel / vehicle speed
    "wheel_speed_fl": "Wheel Speed FL",
    "wheel_speed_fr": "Wheel Speed FR",
    "wheel_speed_rl": "Wheel Speed RL",
    "wheel_speed_rr": "Wheel Speed RR",
    "ground_speed": "MCM GroundSpeed",

    # motor / inverter (MCM)
    "motor_speed": "MCM Motor Speed",
    "motor_temp": "MCM Motor Temperature",
    "torque_feedback": "MCM Torque Feedback",
    "torque_command": "MCM Torque Command",
    "commanded_torque": "MCM Commanded Torque",
    "dc_bus_voltage": "MCM DC Bus Voltage",
    "dc_bus_current": "MCM DC Bus Current",

    # driver inputs
    "throttle_percent": "ThrottlePercent0FF",
    "tps0_percent": "TPS0ThrottlePercent0FF",
    "brake_percent": "BrakePercent0FF",
    "brake_pres_front": "Brake Pres Front",
    "brake_pres_rear": "Brake Pres Rear",

    # pack / BMS
    "bms_soc_percent": "BMS SOC Percent",
    "bms_current": "BMS Current",
    "bms_pack_voltage": "BMS Total Pack Voltage",
    "bms_hv_sense_voltage": "BMS HVsens Pack Voltage",
    "dash_battery_voltage": "Bat Volts Dash",

    # GPS
    "gps_lat": "GPS Latitude",
    "gps_lon": "GPS Longitude",
}


def col(key: str) -> str:
    """Return the MoTeC label for a friendly channel key."""
    try:
        return CHANNELS[key]
    except KeyError as exc:
        raise KeyError(f"Unknown channel key {key!r}. Add it to Data/channels.py") from exc


def cols(*keys: str) -> list[str]:
    """Return MoTeC labels for several friendly keys."""
    return [col(k) for k in keys]


def present(df_columns, keys: list[str]) -> list[str]:
    """Return MoTeC labels for keys that exist in this log's columns."""
    return [col(k) for k in keys if col(k) in df_columns]
