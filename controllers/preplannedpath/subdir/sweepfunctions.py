from controller import Robot, Motor, DistanceSensor, GPS, Emitter, Receiver
from functions import getBearing, getDistanceandRotation, moveTo
import numpy as np

#declare detector angles and distances and angles from gps
ds_1_distance = 0.12
ds_1_angle = 60
ds_1_disp_angle = -30
ds_2_distance = 0.12
ds_2_angle = -60
ds_2_disp_angle = 30

#declare wall ordinates
wall_list_x = [-1.2, 1.2]
wall_list_z = [-1.2, 1.2]

#detect an object

def obstacle_check():
    """
    A function called if the distance sensors detect an object within the sweep lane. It determines whether the object
    is a block or a wall, calling the reciprocating_sweep function if it is a block.
    """
    for i in range(1,2,1):
        #make a rough estimate of coordinates of the point on a surface it 'sees'
        #find absolute angle of detector
        ds_[i]_absolute_angle = getBearing() + ds_[i]_angle
        ds_[i]_absolute_disp_angle = getBearing() + ds_[i]_disp_angle
        #find coordinates
        x_prelim = ds_[i].getValue() * np.sin(ds_[i]_absolute_angle) + ds_[i]_distance * np.sin(ds_[i]_absolute_disp_angle) + get_gps_xz()[0]
        z_prelim = ds_[i].getValue() * np.cos(ds_[i]_absolute_angle) + ds_[i]_distance * np.cos(ds_[i]_absolute_disp_angle) + get_gps_xz()[1]
        prelim_coords = [x_prelim, z_prelim]
        #check if the object is a wall
        if x_prelim in wall_list_x:
            pass
        if z_prelim in wall_list_z:
            pass
        else:
            stop()
            reciprocating_sweep(i)

def reciprocating_sweep(i):
    """
    A function which the robot uses to find the edge of a detected block. It returns the coordinates of the block.

    Arguments: i (the distancesensor detected an obstacle)
    """
    back_coords = [(get_gps_xz()[0] - 0.08 * np.sin(getBearing())), (get_gps_xz()[1] - 0.08 * np.cos(getBearing()))]
    moveto(get_gps_xz(), get_gps_xz(), )
    for d in range(0,80,5):
        #find coordinates
        x_coord = ds_[i].getValue() * np.sin(ds_[i]_absolute_angle) + ds_[i]_distance * np.sin(ds_[i]_absolute_disp_angle) + get_gps_xz()[0]
        z_coord = ds_[i].getValue() * np.cos(ds_[i]_absolute_angle) + ds_[i]_distance * np.cos(ds_[i]_absolute_disp_angle) + get_gps_xz()[1]
        measured_coords = [x_coord, z_coord]
        measured_distance = ds_[i].getValue()
        if d > 0:
            if measured_distance > prev_distance + 0.01:
                block_coords = measured_coords
            else:
                pass
        else:
            pass
        prev_coords = measured_coords
        prev_distance = measured_distance
