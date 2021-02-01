"""preplannedpath controller."""

# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, DistanceSensor, GPS, Emitter, Receiver
from subdir.functions import getBearing, getDistanceandRotation, moveTo, get_gps_xz
import math
import struct #to convert native python data types into a string of bytes and vice versa
import numpy as np

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
TIME_STEP = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:

leftMotor = robot.getDevice('wheel1')
rightMotor = robot.getDevice('wheel2')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

gps = robot.getDevice('gps')
gps.enable(TIME_STEP)

compass = robot.getDevice('compass')
compass.enable(TIME_STEP)

ds_1 = robot.getDevice('ds_left')
ds_1.enable(TIME_STEP)
ds_2 = robot.getDevice('ds_right')
ds_2.enable(TIME_STEP)

emitter = robot.getDevice('emitter')
#emitter.enable(TIME_STEP)

#path = [[1,0,1], [1,0,1],[0,0,1],[0,0,0],[-1,0,0],[-1,0,-1],[0,0,-1],[0,0,0],[-1,0,0],[0,0,1],[1,0,-0.8]] # always duplicate first point
path = [[1,1], [1,1],[0,1],[1,1],[-1,0],[0,0],[-1,-1],[0,0]] # always duplicate first point

i = 0
previous_coordinates = path[0]


# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:

    if i == len(path)-2:
        print('reached the end')
        break

    # get current device values
    current_coordinates = get_gps_xz(gps.getValues())
    north = compass.getValues()
    current_bearing = getBearing(north)
    ds_1_value = ds_1.getValue()
    ds_2_value = ds_2.getValue()

    # detect obstacles
    right_obstacle = ds_1_value < 300.0
    left_obstacle = ds_2_value < 300.0

    #if theres a new point that you want to robot to go to that is not on the original path,
    #insert in the i+2 location in the path
    #example of left obstacle (just an example)
    if current_bearing > 145 and current_bearing < 225: #facing south
        if left_obstacle == True: # set new coordinte to have a reduced z coordinate
            message = struct.pack("3f", *current_coordinates)
            emitter.send(message)
            print('sent', message)
            new_coordinates = [current_coordinates[0], current_coordinates[1]-0.4]
            path.insert(i+2,new_coordinates)
            print('path edited')

    # calculating distance between the desired coordinate and current coordinate
    desired_coordinates = path[i+2]

    MAX_SPEED = 6.28

    leftSpeed, rightSpeed, i = moveTo(previous_coordinates, current_coordinates, desired_coordinates, current_bearing, i)
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    previous_coordinates = current_coordinates

"""
This section of the code details the functions for detecting an object by 'sweeping'
the local area. They are in the main controller program to allow them to retrieve
values from the sensors directly.
"""

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

def obstacle_check(i):
    """
    A function called if the distance sensors detect an object within the sweep lane. It determines whether the object
    is a block or a wall, calling the reciprocating_sweep function if it is a block.

    Arguments: i (the distance sensor that called the function)
    """
    #make a rough estimate of coordinates of the point on a surface it 'sees'
    #find absolute angle of detector
    ds_[i]_absolute_angle = getBearing(compass.getValues()) + ds_[i]_angle
    ds_[i]_absolute_disp_angle = getBearing(compass.getValues()) + ds_[i]_disp_angle
    #find coordinates
    x_prelim = ds_[i].getValue() * np.sin(ds_[i]_absolute_angle) + ds_[i]_distance * np.sin(ds_[i]_absolute_disp_angle) + get_gps_xz(gps.getValues())[0]
    z_prelim = ds_[i].getValue() * np.cos(ds_[i]_absolute_angle) + ds_[i]_distance * np.cos(ds_[i]_absolute_disp_angle) + get_gps_xz(gps.getValues())[1]
    prelim_coords = [x_prelim, z_prelim]
    #check if the object is a wall
    for n in wall_list_x:
        if (wall_list_x[n] - 0.01) <= prelim_coords[0] <= (wall_list_x[n] + 0.01):
            pass
        else:
            stop()
            reciprocating_sweep(i)
    for n in wall_list_z:
        if (wall_list_x[n] - 0.01) <= prelim_coords[1] <= (wall_list_x[n] + 0.01):
            pass
        else:
            stop()
            reciprocating_sweep(i)

def reciprocating_sweep(i):
    """
    A function which the robot uses to find the edge of a detected block. It returns the coordinates of the block.

    Arguments: i (the distance sensor which detected an obstacle)
    """
    #move back 80mm
    #find coordinates 80mm behind
    back_coords = [(get_gps_xz(gps.getValues())[0] - 0.08 * np.sin(getBearing(compass.getValues()))), (get_gps_xz(gps.getValues())[1] - 0.08 * np.cos(getBearing(compass.getValues())))]
    moveto(get_gps_xz(gps.getValues()), get_gps_xz(gps.getValues()), back_coords(), getBearing(), i)
    #move forwards 80mm, 5mm at a time
    for d in range(0, 0.08, 0.005):
        #find coordinates of point on line sensor
        x_coord = ds_[i].getValue() * np.sin(ds_[i]_absolute_angle) + ds_[i]_distance * np.sin(ds_[i]_absolute_disp_angle) + get_gps_xz(gps.getValues())[0]
        z_coord = ds_[i].getValue() * np.cos(ds_[i]_absolute_angle) + ds_[i]_distance * np.cos(ds_[i]_absolute_disp_angle) + get_gps_xz(gps.getValues())[1]
        measured_coords = [x_coord, z_coord]
        measured_distance = ds_[i].getValue()
        #do not check the first time as nothing to compare
        if d > 0:
            #check if a jump occurs in the sensor readings, which indicates the edge
            if measured_distance < prev_distance - 0.01:
                block_coords = measured_coords
                break
            else:
                pass
        else:
            pass
        prev_coords = measured_coords
        prev_distance = measured_distance
    if block_coords != [0, 0]: #any impossible coords will do here
        #route to just in front of block to check colour
        x_check_coord = block_coords[0] - 0.1 * np.sin(getBearing(compass.getValues()))
        z_check_coord = block_coords[1] - 0.1 * np.cos(getBearing(compass.getValues()))
        check_coords = [x_check_coord, z_check_coord]
        moveto(get_gps_xz(), get_gps_xz(), block_coords(), getBearing(compass.getValues()), i)
        #check block colour

    else:
        pass
    block_coords = [0, 0]

def stop():
    """
    This function stops the robot.
    """
    leftMotor.setVelocity(0.0)
    rightMotor.setVelocity(0.0)



# Enter here exit cleanup code.
