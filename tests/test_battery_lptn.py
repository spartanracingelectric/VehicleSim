from pathlib import Path
import sys

if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def plotPowerLimit(show=True):
    import matplotlib.pyplot as plt
    from Vehicle.Battery.Configs.sr17 import create_sr17_battery

    battery = create_sr17_battery(initial_temp_C=25.0, ambient_temp_C=24.75)
    battery.save_history = False
    power_limit_W = 50000.0
    dt_s = 0.1
    duration_s = 300.0
    time_s = [0.0]
    power_kW = [0.0]
    sensor_C = [battery.temp_C]
    pack_C = [battery.pack_node_temp_C]

    for _ in range(round(duration_s / dt_s)):
        current_A = battery.currentForPower(
            power_limit_W, battery.getPackOpenCircuitVoltage_V(),
        )
        battery.update(current_A, dt_s)
        time_s.append(battery.elapsed_time_s)
        power_kW.append(battery.terminal_power_W / 1000.0)
        sensor_C.append(battery.temp_C)
        pack_C.append(battery.pack_node_temp_C)

    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True, layout="constrained")
    axes[0].plot(time_s[1:], power_kW[1:])
    axes[0].set_ylabel("Pack power (kW)")
    axes[0].set_ylim(0, 60)
    axes[0].set_title("50 kW constant")
    axes[1].plot(time_s, sensor_C, label="Thermistor temp")
    axes[1].plot(time_s, pack_C, label="Pack temp")
    axes[1].axhline(60.0, color="red", linestyle="--", label="60")
    axes[1].set_ylabel("Temperature (C)")
    axes[1].set_xlabel("Time (s)")
    axes[1].legend()
    for axis in axes:
        axis.grid(alpha=0.25)
    fig.suptitle("25 C ambient", fontsize=11)
    output = Path(__file__).resolve().parents[1] / "outputs/battery_lptn.png"
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160)
    print(f"Saved graph to {output}")
    if show:
        plt.show()
    else:
        plt.close(fig)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args()
    plotPowerLimit(show=not args.no_show)
