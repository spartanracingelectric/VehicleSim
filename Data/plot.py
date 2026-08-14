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

#TODO: add more plots here in the future

#TODO: save plots to a file


#TODO: show all plots
