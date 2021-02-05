import numpy as np
import tkinter as tk
from tkinter import ttk

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
matplotlib.use("TkAgg")

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from matplotlib import style

LARGE_FONT = ("Verdana", 12)
style.use("ggplot")

class waypoint_generator(tk.Tk):                                # inherit Tk
    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)        
        container = ttk.Frame(self, width=500, height=500)      # initialise window
        container.pack(side="top", fill="both", expand=True)    # pack window
        
        container.grid_rowconfigure(0, weight=1)                
        container.grid_columnconfigure(0, weight=1)             

        self.frames = {}

        for F in (Map, Viz):                                       # Add new pages in for loop
            frame = F(container, self)
            self.frames[F] = frame
            frame.grid(row=0, column=0, sticky="nsew")          # make grid of frames
        
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

        fig = plt.figure(figsize=(5,5), dpi=100)
        ax = plt.gca()

        arena_width = 2.4

        arena   = patches.Rectangle((0,0), arena_width/2, arena_width/2, transform=ax.transAxes, facecolor='slategray')
        red     = patches.Rectangle((0,1-0.15), 0.15, 0.15, transform=ax.transAxes, facecolor='red')
        blue    = patches.Rectangle((0,0), 0.15, 0.15, transform=ax.transAxes, facecolor='blue')
        ax.add_patch(arena)
        ax.add_patch(red)
        ax.add_patch(blue)

        plt.xlim(-arena_width/2, arena_width/2)
        plt.ylim(-arena_width/2, arena_width/2)  
        plt.xticks([-arena_width/2, 0, +arena_width/2])
        plt.yticks([-arena_width/2, +arena_width/2])

        # ax.spines['right'].set_color('none')
        # ax.spines['top'].set_color('none')
        # ax.xaxis.set_ticks_position('bottom')
        # ax.spines['bottom'].set_position(('data',0))
        # ax.yaxis.set_ticks_position('left')
        # ax.spines['left'].set_position(('data',0))

        plt.tight_layout()
        
        canvas = FigureCanvasTkAgg(fig, self)
        canvas.draw()
        canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

class Viz(tk.Frame):
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        label = ttk.Label(self, text="Viz", font=LARGE_FONT)
        label.pack(pady=10, padx=10)

        fig = plt.figure(figsize=(5,5), dpi=100)
        ax = plt.gca()

        arena_width = 2.4

        arena   = patches.Rectangle((0,0), arena_width/2, arena_width/2, transform=ax.transAxes, facecolor='slategray')
        red     = patches.Rectangle((0,1-0.15), 0.15, 0.15, transform=ax.transAxes, facecolor='red')
        blue    = patches.Rectangle((0,0), 0.15, 0.15, transform=ax.transAxes, facecolor='blue')
        ax.add_patch(arena)
        ax.add_patch(red)
        ax.add_patch(blue)

        plt.xlim(-arena_width/2, arena_width/2)
        plt.ylim(-arena_width/2, arena_width/2)  
        plt.xticks([-arena_width/2, 0, +arena_width/2])
        plt.yticks([-arena_width/2, +arena_width/2])

        # ax.spines['right'].set_color('none')
        # ax.spines['top'].set_color('none')
        # ax.xaxis.set_ticks_position('bottom')
        # ax.spines['bottom'].set_position(('data',0))
        # ax.yaxis.set_ticks_position('left')
        # ax.spines['left'].set_position(('data',0))

        plt.tight_layout()
        
        canvas = FigureCanvasTkAgg(fig, self)
        canvas.draw()
        canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

def export_waypoints():
    print("Exported Waypoints")

if __name__ == "__main__":
    app = waypoint_generator()
    app.mainloop()