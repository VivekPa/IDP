"""preplannedpath controller."""

# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, DistanceSensor, GPS
from subdir.functions import getBearing, getDistanceandRotation, moveTo
import math
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

path = [[1,0,1], [1,0,1],[0,0,1],[0,0,0],[-1,0,0],[-1,0,-1],[0,0,-1],[0,0,0],[-1,0,0],[0,0,1],[1,0,-0.8]] # always duplicate first point
i = 0
previous_coordinates = path[0]


# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:

    if i == len(path)-2:
        print('reached the end')
        break

    # get current device values
    current_coordinates = gps.getValues()
    north = compass.getValues()
    current_bearing = getBearing(north)
    ds_leftvalue = ds_left.getValue()
    ds_rightvalue = ds_right.getValue()

    # detect obstacles
    right_obstacle = ds_rightvalue < 300.0
    left_obstacle = ds_leftvalue < 300.0
    
    #if theres a new point that you want to robot to go to that is not on the original path,
    #insert in the i+2 location in the path
    #example of left obstacle (just an example)
    if current_bearing > 145 and current_bearing < 225: #facing south
        if left_obstacle == True: # set new coordinte to have a reduced z coordinate
            new_coordinates = [current_coordinates[0], current_coordinates[1], current_coordinates[2]-0.4]
            path.insert(i+2,new_coordinates)
            print('path edited')

    # calculating distance between the desired coordinate and current coordinate 
    desired_coordinates = path[i+2]
    
    MAX_SPEED = 6.28
    
    leftSpeed, rightSpeed, i = moveTo(previous_coordinates, current_coordinates, desired_coordinates, current_bearing, i)
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    previous_coordinates = current_coordinates

    

    



# Enter here exit cleanup code.
