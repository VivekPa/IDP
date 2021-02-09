from controller import Robot, Motor, DistanceSensor, GPS, Emitter, Receiver, Camera
import math
import numpy as np
import struct

from utils.variables import *
from utils.obstacle_detection import *
from utils.sensors_api import *
from utils.motion_api import *
from utils.obstacle_detection import *

"""Initialise robot"""
#region
robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

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
receiver = robot.getDevice('receiver')
receiver.enable(TIME_STEP)
camera_left = robot.getDevice('camera_left')
camera_left.enable(TIME_STEP)
# camera_left.recognitionEnable(TIME_STEP)
camera_right = robot.getDevice('camera_right')
camera_right.enable(TIME_STEP)
# camera_right.recognitionEnable(TIME_STEP)
#endregion
""""""

while robot.step(TIME_STEP) != -1:
    if i == len(path)-2:
        print('reached the end')

    # get current device values
    current_coordinates = getCoordinates(gps)
    current_bearing = getBearing(compass)
    ds_1_value = ds_left.getValue()
    ds_2_value = ds_right.getValue()

    #region
    #send gps coordinates to other robot
    #message_robot = [0, *current_coordinates] # 0 - robot's coordinates, 1 - block coordinates
    #message_robot = struct.pack("4f", *message_robot)
    #sadly this doesnt work in python 2.7, which george cant stop his computer
    #from using
    #endregion
    
    #message_robot = [0, *current_coordinates] # 0 - robot's coordinates, 1 - block coordinates
    
    message_robot = [0]                                         # Select message type as robot coordinates
    message_robot.extend(current_coordinates)                   # Send coordinates
    message_robot = struct.pack("3f",   message_robot[0],       # Pack message type
                                        message_robot[1],       # Pack x coordinate
                                        message_robot[2])       # Pack z coordinate

    emitter.send(message_robot)

    #receive other robot's coordinates
    #print('Receiver Queue length:'  , receiver.getQueueLength())
    if receiver.getQueueLength() > 0:
        message = receiver.getData()
        message = list(struct.unpack("3f",message))
        if message[0] == 0:
            other_robot_coordinates = message[1:]
        elif message[0] == 1:
            np.append(block_coords, message[1:])

        print('Blue robot location:', message)
        receiver.nextPacket() #deletes the head packet
    # Process sensor data here.
    # else:
        # print('no message')

    distance_btw_robots = np.linalg.norm(np.array(current_coordinates) - np.array(other_robot_coordinates))
    # print(distance_btw_robots)
    # detect obstacles
    right_obstacle = ds_1_value < 1000.0
    left_obstacle = ds_2_value < 1000.0

    # Check if the robot is near home
    if np.linalg.norm(np.array(current_coordinates) - np.array(home)) < 0.1:
        atHome = True
    else:
        atHome = False

    """ Unloading """
    #region
    if atHome and robot.getTime() > 8:  # If the robot is near home after 'return_to_home()'
        unloading = True                # Initiate unloading procedure
        reverse_coords = calc_reverse_coords(current_coordinates, current_bearing)

    #call obstacle_check only if obstacle outside of while loop and unloading is false
    #potentially changes obstacle to True if it detects a block
    if obstacle == False and unloading == False:
        if right_obstacle == True:
            # obstacle_coords = find_obstacle_coords(ds_left, gps,compass)

            block_coords, obstacle = obstacle_check(ds_left, gps, compass, obstacle, other_colour_blocks)
            last_known_point = getCoordinates(gps)
        else:
            pass
        if left_obstacle == True:
            block_coords, obstacle = obstacle_check(ds_right, gps, compass, obstacle, other_colour_blocks)
            last_known_point = getCoordinates(gps)
            # print(last_known_point)
        else:
            pass
    else:
        pass

    #endregion

    #if theres a new point that you want to robot to go to that is not on the original path,
    #insert in the i+2 location in the path
    #example of left obstacle (just an example)
    """
    if current_bearing > 145 and current_bearing < 225: #facing south
        if left_obstacle == True: # set new coordinte to have a reduced z coordinate
            message = struct.pack("3f", *current_coordinates)
            emitter.send(message)
            print('sent', message)
            new_coordinates = [current_coordinates[0], current_coordinates[2]-0.4]
            path.insert(i+2,new_coordinates)
            print('path edited')
    """
    # calculating distance between the desired coordinate and current coordinate
    desired_coordinates = path[i+2]

 #check if robot has made a turn
    coordinates = path[i]
    if coordinates in turnpoints:
        path_turns += 1
        path = np.delete(path,-1, axis=0)
        print('turned')

    if unloading == False:
        #obstacle Boolean here might be different from the obstacle boolean at the start of this loop due to the previous if statement
        if obstacle == True and goinghome == False:
            alignment = False
            leftSpeed, rightSpeed, alignment = rotateTo(previous_coordinates, current_coordinates, block_coords, current_bearing, alignment)
            if alignment == True:
                colour = getRGB(camera_left, camera_right)
                alignment = False #switch alignment back to false
                # print(colour)
                if colour == robot_colour: #implement collection function
                    # print('yeboi collect it')
                    leftSpeed, rightSpeed, j = moveTo(previous_coordinates, current_coordinates, block_coords, current_bearing, i)
                    if j == i+1: #collected block
                        obstacle = False
                        goinghome = True
                        # print(type(last_known_point[0]))
                        # print(type(cartesian_bearing))
                        # path.insert(i+2, [last_known_point[0] + 0.1 * np.cos(cartesian_bearing * deg2rad), last_known_point[1] + 0.1 * np.sin(cartesian_bearing * deg2rad)])
                        print(i)
                        print(path.size)
                        print(last_known_point)
                        path = np.insert(path, i+2, 
                                        [last_known_point[0] + 0.1 * np.cos(cartesian_bearing * deg2rad),
                                         last_known_point[1] + 0.1 * np.sin(cartesian_bearing * deg2rad)],
                                         axis=0)
                        if path_turns == 0:
                            # path.insert(i+3, home)
                            path = np.insert(path, i+3, home, axis=0)
                        else:
                            # path.insert(i+3, [last_known_point[0], 1])
                            # path.insert(i+4, home)
                            path = np.insert(path, i+3, [last_known_point[0], 1], axis=0)
                            path = np.insert(path, i+4, home, axis=0)

                elif colour == other_robot_colour: #implement avoidance function
                    print('nah screw you')
                    new_path_coords = obstacle.avoid_obstacle(current_coordinates, block_coords)
                    path = np.insert(path, i+2, new_path_coords, axis=0)
                elif colour == None:
                    print('cant determine')
                    # leftSpeed  = 0
                    # rightSpeed = 0

                #obstacle = False #change obstacle back to False after collecting the block
                #print(obstacle)
                # leftMotor.setVelocity(leftSpeed)
                # rightMotor.setVelocity(rightSpeed)
                # print('trying to break')
                # break
        # elif obstacle == True and goinghome == True:
        #     print('trying to avoid')
        #     #implement avoidance function
        else:
            # print(desired_coordinates)
            # print(current_coordinates)
            # print(previous_coordinates)
            leftSpeed, rightSpeed, i = moveTo(previous_coordinates, current_coordinates, desired_coordinates, current_bearing, i)
    elif unloading == True:
        leftSpeed, rightSpeed, j = reverseTo(previous_coordinates, current_coordinates, reverse_coords, i)
        if j == i + 1:
            i += 1
            #finish unloading, reset all state variables
            goinghome = False
            unloading = False
            obstacle = False

    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    previous_coordinates = current_coordinates
