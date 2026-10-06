#this is for plotting data only, strictly no calculations should be done here
import matplotlib.pyplot as plt


def plot(xlist: list, ylist: list, xlabel: str, ylabel: str, title: str):
    print(xlist)
    print(ylist)
    
    plt.plot(xlist, ylist, marker="o")
    plt.xlabel("xlabel")
    plt.ylabel("ylabel")
    plt.title("title")
    plt.grid(True)
    plt.show()

# Level 2 inverter: carrier comparison, switched vs average leg voltage, phase currents, dc current
def plotPwm(time_s, carrier, duty_cycles, switched_voltages_V, average_voltages_V,
            phase_currents_A, average_currents_A, dc_current_A, average_dc_current_A,
            zoom_window_s, save_path=None):
    phases = ["a", "b", "c"]
    time_ms = time_s * 1000
    zoom_ms = (zoom_window_s[0] * 1000, zoom_window_s[1] * 1000)
    fig, axes = plt.subplots(4, 1, figsize=(11, 13))

    ax = axes[0]
    ax.plot(time_ms, carrier, color="gray", lw=1, label="carrier")
    for p in range(3):
        ax.step(time_ms, duty_cycles[:, p], where="post", color=f"C{p}", label=f"duty {phases[p]}")
    ax.set_xlim(zoom_ms)
    ax.set_ylabel("Duty / carrier")
    ax.set_title("Carrier comparison: upper IGBT on while duty > carrier")

    ax = axes[1]
    ax.step(time_ms, switched_voltages_V[:, 0], where="post", color="C0", lw=1, label="switched")
    ax.step(time_ms, average_voltages_V[:, 0], where="post", color="black", ls="--", label="average (Level 1)")
    ax.set_xlim(zoom_ms)
    ax.set_ylabel("Phase a leg voltage (V)")
    ax.set_title("Phase a leg: switched between 0 and V_bus, current ripples with each switch")

    # phase a current on a second axis, scaled to the zoom window so the ripple shows
    in_zoom = (time_ms >= zoom_ms[0]) & (time_ms <= zoom_ms[1])
    current_ax = ax.twinx()
    current_ax.plot(time_ms, phase_currents_A[:, 0], color="C3", lw=1.2, label="i_a switched")
    current_ax.plot(time_ms, average_currents_A[:, 0], color="C3", ls=":", lw=1.2, label="i_a average model")
    current_ax.set_xlim(zoom_ms)
    current_ax.set_ylim(phase_currents_A[in_zoom, 0].min() - 1, phase_currents_A[in_zoom, 0].max() + 1)
    current_ax.set_ylabel("Phase a current (A)")
    current_ax.legend(loc="lower right", fontsize=8)

    ax = axes[2]
    for p in range(3):
        ax.plot(time_ms, phase_currents_A[:, p], color=f"C{p}", lw=0.8, label=f"i_{phases[p]} switched")
        ax.plot(time_ms, average_currents_A[:, p], color="black", ls="--", lw=0.8,
                label="average model" if p == 0 else None)
    ax.axvspan(*zoom_ms, color="gray", alpha=0.2)
    ax.set_ylabel("Phase current (A)")
    ax.set_title("Phase currents: load inductance smooths the switching into AC")

    ax = axes[3]
    ax.plot(time_ms, dc_current_A, color="C3", lw=0.4, alpha=0.6, label="switched")
    ax.step(time_ms, average_dc_current_A, where="post", color="black", lw=1.2, label="per-period average")
    ax.axvspan(*zoom_ms, color="gray", alpha=0.2)
    ax.set_ylabel("DC bus current (A)")
    ax.set_xlabel("Time (ms)")
    ax.set_title("DC bus current: sum of the phase currents switched to DC+")

    for ax in axes:
        ax.grid(True)
        ax.legend(loc="upper right", fontsize=8)

    fig.tight_layout()
    if save_path is None:
        plt.show()
    else:
        fig.savefig(save_path, dpi=110)
    plt.close(fig)

# motor + inverter: current tracking, torque, voltage vs inverter limit, power and dc current
def plotDrive(time_s, torque_request_Nm, torque_Nm, iq_desired_A, iq_A, id_A,
              vd_V, vq_V, voltage_magnitude_V, max_voltage_V,
              mech_power_W, motor_power_W, dc_power_W, dc_current_A, title, save_path=None):
    time_ms = time_s * 1000
    fig, axes = plt.subplots(4, 1, figsize=(11, 13), sharex=True)
    fig.suptitle(f"Motor + inverter: {title}")

    ax = axes[0]
    ax.plot(time_ms, iq_desired_A, color="black", ls="--", lw=1, label="iq desired")
    ax.plot(time_ms, iq_A, color="C0", label="iq")
    ax.plot(time_ms, id_A, color="C1", label="id (desired 0)")
    ax.set_ylabel("Current (A)")
    ax.set_title("PI controller: rotor-frame currents")

    ax = axes[1]
    ax.plot(time_ms, torque_request_Nm, color="black", ls="--", lw=1, label="request")
    ax.plot(time_ms, torque_Nm, color="C2", label="motor torque")
    ax.set_ylabel("Torque (Nm)")
    ax.set_title("Torque")

    ax = axes[2]
    ax.plot(time_ms, vd_V, color="C1", lw=1, label="vd")
    ax.plot(time_ms, vq_V, color="C0", lw=1, label="vq")
    ax.plot(time_ms, voltage_magnitude_V, color="C3", label="|v|")
    ax.plot(time_ms, max_voltage_V, color="black", ls="--", lw=1, label="inverter limit")
    ax.set_ylabel("Voltage (V)")
    ax.set_title("Voltage the inverter applies, seen from the rotor")

    ax = axes[3]
    ax.plot(time_ms, mech_power_W / 1000, color="C2", label="shaft")
    ax.plot(time_ms, motor_power_W / 1000, color="C0", label="motor electrical")
    ax.plot(time_ms, dc_power_W / 1000, color="C3", label="battery (dc)")
    ax.set_ylabel("Power (kW)")
    ax.set_xlabel("Time (ms)")
    ax.set_title("Power: shaft < motor electrical < battery when motoring")
    current_ax = ax.twinx()
    current_ax.plot(time_ms, dc_current_A, color="gray", ls=":", label="dc current")
    current_ax.set_ylabel("DC current (A)")
    current_ax.legend(loc="lower right", fontsize=8)

    for ax in axes:
        ax.grid(True)
        ax.legend(loc="upper right", fontsize=8)

    fig.tight_layout()
    if save_path is None:
        plt.show()
    else:
        fig.savefig(save_path, dpi=110)
    plt.close(fig)

# acceleration run vs time: pedal and torque, car speed and motor rpm, pack power vs the bms overpower flag,
# pack voltage and current
def plotAcceleration(time_s, apps_percent, torque_request_Nm, torque_feedback_Nm, speed_kph, motor_rpm,
                     pack_power_kW, power_threshold_kW, overpower, pack_voltage_V, pack_current_A, title, save_path=None):
    fig, axes = plt.subplots(4, 1, figsize=(11, 14), sharex=True)
    fig.suptitle(f"Straight-line run: {title}")

    ax = axes[0]
    ax.step(time_s, torque_request_Nm, where="post", color="black", ls="--", lw=1, label="vcu request (every 10 ms)")
    ax.plot(time_s, torque_feedback_Nm, color="C2", label="torque feedback (what the motor made)")
    ax.set_ylim(bottom=0, top=max(torque_request_Nm) * 1.3)
    ax.set_ylabel("Torque per motor (Nm)")
    ax.set_title("Pedal and torque")
    pedal_ax = ax.twinx()
    pedal_ax.plot(time_s, apps_percent, color="gray", ls=":", label="pedal (APPS)")
    pedal_ax.set_ylim(0, 110)
    pedal_ax.set_ylabel("Pedal (%)")
    pedal_ax.legend(loc="lower right", fontsize=8)

    ax = axes[1]
    ax.plot(time_s, speed_kph, color="C0", label="car speed")
    ax.set_ylabel("Speed (km/h)")
    ax.set_title("Car speed and motor speed")
    rpm_ax = ax.twinx()
    rpm_ax.plot(time_s, motor_rpm, color="C1", ls=":", label="motor rpm")
    rpm_ax.set_ylabel("Motor speed (rpm)")
    rpm_ax.legend(loc="lower right", fontsize=8)

    ax = axes[2]
    ax.fill_between(time_s, 0, 1, where=overpower, color="C3", alpha=0.15,
                    transform=ax.get_xaxis_transform(), label="bms overpower flag")
    ax.plot(time_s, power_threshold_kW, color="black", ls="--", lw=1, label="overpower threshold")
    ax.plot(time_s, pack_power_kW, color="C3", label="pack power")
    ax.set_ylabel("Power (kW)")
    ax.set_title("Pack power (not limited here, the VCU does that)")

    ax = axes[3]
    ax.plot(time_s, pack_voltage_V, color="C0", label="pack voltage")
    ax.set_ylabel("Pack voltage (V)")
    ax.set_xlabel("Time (s)")
    ax.set_title("Pack voltage sags as the current goes up")
    current_ax = ax.twinx()
    current_ax.plot(time_s, pack_current_A, color="C3", ls=":", label="pack current")
    current_ax.set_ylabel("Pack current (A)")
    current_ax.legend(loc="lower right", fontsize=8)

    for ax in axes:
        ax.grid(True)
        ax.legend(loc="upper left", fontsize=8)

    fig.tight_layout()
    if save_path is None:
        plt.show()
    else:
        fig.savefig(save_path, dpi=110)
    plt.close(fig)