"""Load MoTeC CSV logs into time/speed arrays for the sim."""

from pathlib import Path

import numpy as np
import pandas as pd

from Data.channels import CHANNELS, col, present
from Functions.constants import MPH_TO_MPS


def find_motec_header_row(path: Path) -> int:
    """Find the 0-based row index where channel names start (first cell == 'Time')."""
    time_label = col("time")
    with path.open(newline="", encoding="utf-8", errors="replace") as f:
        for i, line in enumerate(f):
            # MoTeC quotes fields: "Time","G Force Lat DAQ",...
            first = line.strip().lstrip('"').split('"', 1)[0].split(",", 1)[0].strip()
            if first == time_label:
                return i
    raise ValueError(f"Could not find MoTeC channel header ({time_label}) in {path}")


def load_motec_csv(path: str | Path) -> pd.DataFrame:
    """Read a MoTeC CSV and return numeric channel data."""
    path = Path(path)
    header_row = find_motec_header_row(path)
    df = pd.read_csv(path, skiprows=header_row, low_memory=False)

    time_col = col("time")
    df[time_col] = pd.to_numeric(df[time_col], errors="coerce")
    df = df.dropna(subset=[time_col]).reset_index(drop=True)

    for label in CHANNELS.values():
        if label in df.columns and label != time_col:
            df[label] = pd.to_numeric(df[label], errors="coerce")

    return df


def load_speed_trace(path: str | Path) -> tuple[np.ndarray, np.ndarray]:
    """Return (time_s, vel_mps) from MoTeC wheel speeds (mph -> m/s)."""
    df = load_motec_csv(path)
    speed_cols = present(
        df.columns,
        [
            "wheel_speed_fl",
            "wheel_speed_fr",
            "wheel_speed_rl",
            "wheel_speed_rr",
        ],
    )
    if not speed_cols:
        raise ValueError(f"No wheel-speed columns found in {path}")

    time_s = df[col("time")].to_numpy(dtype=float)
    vel_mph = df[speed_cols].mean(axis=1).to_numpy(dtype=float)
    vel_mps = vel_mph * MPH_TO_MPS
    return time_s, vel_mps
