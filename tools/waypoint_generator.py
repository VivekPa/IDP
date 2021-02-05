import numpy as np
import tkinter as tk
from tkinter import ttk

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

LARGE_FONT = ("Verdana", 12)

class waypoint_generator(tk.Tk):                                # inherit Tk
    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)        
        container = ttk.Frame(self, width=500, height=500)      # initialise window
        container.pack(side="top", fill="both", expand=True)    # pack window
        
        container.grid_rowconfigure(0, weight=1)                
        container.grid_columnconfigure(0, weight=1)             

        self.frames = {}

        # for F in (Map):                                       # Add new pages in for loop
        frame = Map(container, self)
        self.frames[Map] = frame
        frame.grid(row=0, column=0, sticky="nsew")              # make grid of frames

        self.show_frame(Map)

    def show_frame(self, controller):
        frame = self.frames[controller]
        frame.tkraise()

class Map(tk.Frame):
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        label = ttk.Label(self, text="Map", font=LARGE_FONT)
        label.pack(pady=10, padx=10)

        button1 = ttk.Button(self, text="Export Waypoints", command=export_waypoints())
        button1.pack()

        fig = Figure(figsize=(5,5), dpi=100)
        a = fig.add_subplot(111)              # 1st plot in a 1x1 grid
        rectangle = a.Rectangle((0,0), 50, 20, fc='blue')
        plt.gca().add_patch(rectangle)
        # a.plot([1,2,3,4,5,6,7,8],[5,6,1,2,3,5,3,8])

        canvas = FigureCanvasTkAgg(fig, self)
        canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

def export_waypoints():
    print("Exported Waypoints")

if __name__ == "__main__":
    app = waypoint_generator()
    app.mainloop()