"""preplannedpath controller."""

# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, GPS
from subdir.functions import getBearing, getDistanceandRotation
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


path = [[1,0,1], [1,0,1],[0,0,1],[0,0,0],[-1,0,0],[-1,0,-1],[0,0,-1],[0,0,0],[-1,0,0],[0,0,1],[1,0,-0.8]] # always duplicate first point
i = 0
previous_coordinates = path[0]


# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:

    if i == len(path)-2:
        print('reached the end')
        break
    # calculating distance between the desired coordinate and current coordinate 
    desired_coordinates = path[i+2]
    current_coordinates = gps.getValues()
    coordinates_list = [previous_coordinates,current_coordinates,desired_coordinates]
    distance, x = getDistanceandRotation(coordinates_list)

    # calculating desired bearing to get to desired coordinate from current coordinate 
    ref_coordinates = [current_coordinates[0]+1, current_coordinates[1] , current_coordinates[2]] # to make previous vector always be [-1,0,0]
    coordinates_list2 = [ref_coordinates,current_coordinates,desired_coordinates]
    x,angle = getDistanceandRotation(coordinates_list2)
    print('angle',angle)
    desired_bearing = angle + 180 
    
    print(i,distance, desired_bearing)

    north = compass.getValues()
    current_bearing = getBearing(north)
    bearing_error = desired_bearing - current_bearing
    print('bearing error', bearing_error)
    MAX_SPEED = 6.28
    
    if bearing_error > 180:
        bearing_error -= 360 
    elif bearing_error < -180:
        bearing_error += 360

    if bearing_error < 1 and bearing_error > -1:
        if distance < 0.1:
            leftSpeed  = 0
            rightSpeed = 0
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            i += 1
        else:
            leftSpeed  = 0.5 * MAX_SPEED
            rightSpeed = 0.5 * MAX_SPEED
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
    elif bearing_error >= 1: #rotate right
            leftSpeed  = 0.5 * MAX_SPEED
            rightSpeed = -0.5 * MAX_SPEED
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
    elif bearing_error <= -1: #rotate left
            leftSpeed  = -0.5 * MAX_SPEED
            rightSpeed = 0.5 * MAX_SPEED
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)

    previous_coordinates = current_coordinates

    

    



# Enter here exit cleanup code.
