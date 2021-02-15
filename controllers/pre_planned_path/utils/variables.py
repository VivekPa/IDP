import numpy as np
import os
import csv

"""Robot Colour"""
#region
robot_colour = 0                 # 0 - red, 1 - green, 2- blue
other_robot_colour = 2
#endregion

"""Define Waypoints and Home"""
#region
red_base    = [1,1]
red_standby = [1,0.7]
blue_base   = [1,-1]
blue_standby = [1,-0.7]
home        = red_base
standby     = red_standby
other_robot_coordinates = blue_base

path = np.array([home,home])
og_path = [home,home]
a = 0       # Path index
previous_coordinates = path[0]

dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, '../paths/horizontal_traverse.txt')

with open(filename, 'r') as csvfile:
    csvreader = csv.reader(csvfile,  delimiter = ',')
    for row in csvreader:
        for j in range(len(row)):
            row[j] = float(row[j])
        point = (np.array(row))
        list_point = list(point)
        path = np.vstack([path, point])
        og_path.append(list_point)

timeout = 300    # Simulation time in seconds when robot quits everything to go home
#endregion
"""Path Planning Variables"""

# start and goal position
sx = 100  # [cm]
sy = 100  # [cm]
gx = -100  # [cm]
gy = 100 # [cm]

# set obstacle positions
ox, oy = [], []
exceed= 2 #12.5 - 7.5
#wall boundaries
for i in range(-120-exceed, 121+exceed):
    ox.append(i)
    oy.append(-120-exceed)
for i in range(-120-exceed, 121+exceed):
    ox.append(120+exceed)
    oy.append(i)
for i in range(-120-exceed, 121+exceed):
    ox.append(i)
    oy.append(120+exceed)
for i in range(-120-exceed, 121+exceed):
    ox.append(-120-exceed)
    oy.append(i)

"""State Variables"""
#region
unloading   = False         # Unloading state
obstacle    = False         # Obstacle detection state
obstacle1   = False
obstacle2   = False
goinghome   = False         # Going Home state
atHome      = True          # At Home state
blockcoords_sent = False
colour_determined = False
getting_away = False
other_robot_done = False
started_collecting = False
robot_final_done = False

#endregion

"""Miscellaneous Variables"""
#region
deg2rad     = 3.14159/180
#endregion

"""Constant Variables"""
#region
block_width = 5
wall_list_x = [-1.2, 1.2]
wall_list_z = [-1.2, 1.2]

MAX_SPEED = 10
#endregion

"""Obstacle variables"""
#region
#initialise 'active block coordinates'
list_of_blocks = []
#initialise block list
other_colour_blocks = []
indetermined_obs_blocks = []
#declare last cartesian bearing
cartesian_bearing = 0
#declare last known point
last_known_point = np.array([])
#declare turn variable to decide on path home
path_turns = 0
turnpoints = np.array([[-1, 1.0], [-1, 0.6],[5,5]]) #have a dummy at the end
#endregion
