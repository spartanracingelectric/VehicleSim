#this is for plotting data only, strictly no calculations should be done here
import matplotlib.pyplot as plt


def plot_energy_mass(sim):
    print(sim.total_power_KwHr)
    print(sim.battery_mass_type)

    plt.plot(sim.battery_mass_type, sim.total_power_KwHr, marker="o")
    plt.xlabel("Battery mass (kg)")
    plt.ylabel("Energy (kWh)")
    plt.title("Energy vs battery mass")
    plt.grid(True)
    plt.show()

#TODO: add more plots here in the future

#TODO: save plots to a file


#TODO: show all plots
def plot_all(sim):
    plot_energy_mass(sim)