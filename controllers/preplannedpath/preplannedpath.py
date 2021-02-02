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

ds_left = robot.getDevice('ds_left')
ds_left.enable(TIME_STEP)
ds_right = robot.getDevice('ds_right')
ds_right.enable(TIME_STEP)

emitter = robot.getDevice('emitter')
#emitter.enable(TIME_STEP)

path = [[1,0,1], [1,0,1],[0,0,1],[0,0,0],[-1,0,0],[-1,0,-1],[0,0,-1],[0,0,0],[-1,0,0],[0,0,1],[1,0,-0.8]] # always duplicate first point
#path = [[1,0,1], [1,0,1],[0,0,1],[-1,0,1],[-1,0,0.6],[0,0,0.6],[0,0,0],[-1,0,0],[-1,0,-0.6],[0,0,-0.6]] # always duplicate first point
#path = [[1,1], [1,1],[0,1],[1,1],[-1,0],[0,0],[-1,-1],[0,0]] # always duplicate first point

i = 0
previous_coordinates = path[0]

"""
This section of the code details the functions for detecting an object by 'sweeping'
the local area. They are in the main controller program to allow them to retrieve
values from the sensors directly.
"""

#declare detector angles and distances and angles from gps
def get_attributes(ds):
    """
    A function which returns the distance sensor location and orientation values
    given the name of a distance sensor. Workaround for functions not taking
    objects as arguments.

    Arguments: ds (a string which denotes the desired distance sensor)
    """
    if ds == 'ds_1':
        ds_distance = 0.11
        ds_angle = 60
        ds_disp_angle = -39.8
    elif ds == 'ds_2':
        ds_distance = 0.11
        ds_angle = -60
        ds_disp_angle = 39.8
    else:
        print("distance sensor not found")
    return [ds_distance, ds_angle, ds_disp_angle]


#declare wall ordinates
wall_list_x = [-1.2, 1.2]
wall_list_z = [-1.2, 1.2]

def ds_read(ds):
    """
    A function which returns the correct distance sensor reading given the name
    of a distance sensor. Workaround for functions not taking objects as
    arguments.

    Arguments: ds (a string which denotes the desired distance sensor)
    """
    if ds == 'ds_1':
        ds_value = 0.0003 * ds_left.getValue()
    elif ds == 'ds_2':
        ds_value = 0.0003 * ds_right.getValue()
    else:
        print('distance sensor not found')
    return ds_value

def obstacle_coords(ds):
    """
    A function which returns the coordinates on the obstacle surface which
    the distance sensor is detecting.

    Arguments: ds (a string which denotes the distance sensor which detected an obstacle)
    """
    #make a rough estimate of coordinates of the point on a surface it 'sees'
    #retrieve attributes
    ds_attributes = get_attributes(ds)
    ds_distance = ds_attributes[0]
    #find absolute angle of detector
    ds_absolute_angle = getBearing(compass.getValues()) + ds_attributes[1]
    ds_absolute_disp_angle = getBearing(compass.getValues()) + ds_attributes[2]
    #find coordinates
    x_coord = ds_read(ds) * np.sin(ds_absolute_angle) - ds_distance * np.sin(ds_absolute_disp_angle) + gps.getValues()[0]
    z_coord = ds_read(ds) * np.cos(ds_absolute_angle) - ds_distance * np.cos(ds_absolute_disp_angle) + gps.getValues()[2]
    return x_coord, z_coord

def obstacle_check(ds):
    """
    A function called if the distance sensors detect an object within the sweep lane. It determines whether the object
    is a block or a wall, calling the reciprocating_sweep function if it is a block.

    Arguments: ds (a string which denotes the distance sensor which detected an obstacle)
    """
    x_prelim, z_prelim = obstacle_coords(ds)
    prelim_coords = [x_prelim, z_prelim]
    print(prelim_coords)
    #check if the object is a wall, by comparing with the lines x = 1.2/-1.2,
    #z = 1.2/-1.2
    wall_coord = 1.2
    wall_tolerance = 0.01
    lower_wall = wall_coord - wall_tolerance
    upper_wall = wall_coord + wall_tolerance
    if lower_wall <= abs(x_prelim) <= upper_wall or lower_wall <= abs(z_prelim) <= upper_wall:
        pass
    else:
        print('thats no moon!')
        stop()
        reciprocating_sweep(ds)

def reciprocating_sweep(ds):
    """
    A function which the robot uses to find the edge of a detected block. It returns the coordinates of the block.

    Arguments: ds (a string which denotes the distance sensor which detected an obstacle)
    """
    #Retrieve attributes
    ds_attributes = get_attributes(ds)
    ds_distance = ds_attributes[0]
    ds_absolute_angle = getBearing(compass.getValues()) + ds_attributes[1]
    ds_absolute_disp_angle = getBearing(compass.getValues()) + ds_attributes[2]
    #move back 80mm
    #find coordinates 80mm behind
    back_coords = [(gps.getValues()[0] - 0.08 * np.sin(getBearing(compass.getValues()))), (gps.getValues()[2] - 0.08 * np.cos(getBearing(compass.getValues())))]
    moveTo(get_gps_xz(gps.getValues()), get_gps_xz(gps.getValues()), back_coords(), getBearing(), i)
    #move forwards 80mm, 5mm at a time
    for d in range(0, 0.08, 0.005):
        #find coordinates of point on line sensor
        x_coord = ds_read(ds) * np.sin(ds_absolute_angle) + ds_distance * np.sin(ds_absolute_disp_angle) + get_gps_xz(gps.getValues())[0]
        z_coord = ds_read(ds) * np.cos(ds_absolute_angle) + ds_distance * np.cos(ds_absolute_disp_angle) + get_gps_xz(gps.getValues())[1]
        measured_coords = [x_coord, z_coord]
        measured_distance = ds_read(ds)
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

"""
End of search functions
"""

# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:

    if i == len(path)-2:
        print('reached the end')
        break

    # get current device values
    current_coordinates = gps.getValues()
    north = compass.getValues()
    current_bearing = getBearing(north)
    ds_1_value = ds_left.getValue()
    ds_2_value = ds_right.getValue()

    # detect obstacles
    right_obstacle = ds_1_value < 1000.0
    left_obstacle = ds_2_value < 1000.0

    #call obstacle_check if necessary
    if right_obstacle == True:
        obstacle_check('ds_1')
    elif left_obstacle == True:
        obstacle_check('ds_2')
    else:
        pass

    #if theres a new point that you want to robot to go to that is not on the original path,
    #insert in the i+2 location in the path
    #example of left obstacle (just an example)
    # if current_bearing > 145 and current_bearing < 225: #facing south
    #     if left_obstacle == True: # set new coordinte to have a reduced z coordinate
    #         message = struct.pack("3f", *current_coordinates)
    #         emitter.send(message)
    #         print('sent', message)
    #         new_coordinates = [current_coordinates[0], current_coordinates[2]-0.4]
    #         path.insert(i+2,new_coordinates)
    #         print('path edited')

    # calculating distance between the desired coordinate and current coordinate
    desired_coordinates = path[i+2]

    MAX_SPEED = 6.28

    leftSpeed, rightSpeed, i = moveTo(previous_coordinates, current_coordinates, desired_coordinates, current_bearing, i)
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    previous_coordinates = current_coordinates

# Enter here exit cleanup code.
