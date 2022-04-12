from datalog import load_datalog
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import *

log = load_datalog("robotlog.log")
fields = log.data.keys()
to_plot = []


#Select which fields to plot
willPlot = []
selW = tk.Tk()
selW.title("Select Fields to Plot")

def plot():
    for i in range(len(fields)):
        if (willPlot[i].get()):
            p, = plt.plot(log.time, log.data[list(fields)[i]])
            p.set_label(list(fields)[i])
            plt.legend()

    plt.xlabel("Time")
    plt.ylabel("Value")
    plt.show()

for i in range(len(fields)):
    b = BooleanVar()
    willPlot.append(b)
    Checkbutton(selW, text=list(fields)[i], variable=b, height=2, width=20).grid(row = i, sticky=W)
Button(selW, text="Plot", command=plot).grid(row = len(fields), sticky=W)

selW.mainloop()

