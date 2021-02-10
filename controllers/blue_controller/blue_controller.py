from controller import Robot, Motor, DistanceSensor, GPS, Emitter, Receiver, Camera
import math
import numpy as np
import struct

from utils.variables import *
from utils.obstacle_detection import *
from utils.sensors_api import *
from utils.motion_api import *

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

camera_right = robot.getDevice('camera_right')
camera_right.enable(TIME_STEP)

#endregion

""""""
# print(i)
# print(path_turns)

while robot.step(TIME_STEP) != -1:
    if i == len(path)-2:
        print('reached the end')

    # get current device values
    current_coordinates = getCoordinates(gps)
    current_bearing = getBearing(compass)
    ds_1_value = ds_left.getValue()
    ds_2_value = ds_right.getValue()

    #message_robot = [0, *current_coordinates] # 0 - robot's coordinates, 1 - block coordinates
    message_robot = [0]                                         # Select message type as robot coordinates
    message_robot.extend(current_coordinates)                   # Send coordinates
    message_robot = struct.pack("3f",   message_robot[0],       # Pack message type
                                        message_robot[1],       # Pack x coordinate
                                        message_robot[2])       # Pack z coordinate

    emitter.send(message_robot)

    #region
    #receive other robot's coordinates
    #print('Receiver Queue length:'  , receiver.getQueueLength())
    while receiver.getQueueLength() > 0:
        message = receiver.getData()
        message = np.array(list(struct.unpack("3f",message)))
        if message[0] == 0:
            other_robot_coordinates = message[1:]
        elif message[0] == 1:
            np.append(list_of_blocks, message[1:])

        # print('Red robot location:', other_robot_coordinates)
        receiver.nextPacket() #deletes the head packet
    
    # if receiver.getQueueLength() == 0:
    #     print('no message')
    #endregion


    """Check if Home"""
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
    #endregion


    distance_btw_robots = np.linalg.norm(np.array(current_coordinates) - np.array(other_robot_coordinates))
    # print('distance', distance_btw_robots)
    
    if distance_btw_robots > 2*0.2:
        # detect obstacles
        right_obstacle = ds_1_value < 1000.0
        left_obstacle = ds_2_value < 1000.0

        #call obstacle_check only if obstacle outside of while loop and unloading is false
        #potentially changes obstacle to True if it detects a block
        if obstacle == False and unloading == False:
            if right_obstacle == True:
                # obstacle_coords = find_obstacle_coords(ds_left, gps,compass)
                block_coords, obstacle = obstacle_check(ds_left, gps, compass, obstacle, other_colour_blocks)
                #last_known_point = getCoordinates(gps)
            else:
                pass
            if left_obstacle == True:
                block_coords, obstacle = obstacle_check(ds_right, gps, compass, obstacle, other_colour_blocks)
                #last_known_point = getCoordinates(gps)
                # print(last_known_point)
            else:
                pass
        else:
            pass

        # calculating distance between the desired coordinate and current coordinate
        if len(path) > i+2:
            desired_coordinates = path[i+2]

        #check if robot has made a turn
        if len(path) > i+2:
            coordinates = path[i+1]
        
        # print(path_turns, coordinates)
        if coordinates[0] == turnpoints[0][0] and coordinates[1] == turnpoints[0][1]:
            path_turns += 1
            # print(coordinates,turnpoints)
            turnpoints = np.delete(turnpoints,0, axis=0)
            print(turnpoints)
            print('turned')

        if unloading == False:
            #obstacle Boolean here might be different from the obstacle boolean at the start of this loop due to the previous if statement
            if obstacle == True and goinghome == False:
                alignment = False
                leftSpeed, rightSpeed, alignment = rotateTo(previous_coordinates, current_coordinates, block_coords, current_bearing, alignment)
                if alignment == True:
                    colour = getRGB(camera_left, camera_right)
                    alignment = False #switch alignment back to false
                    print(colour)
                    if colour == robot_colour: #implement collection function
                        print('yeboi collect it')
                        leftSpeed, rightSpeed, j = moveTo(previous_coordinates, current_coordinates, block_coords, current_bearing, i)
                        if j == i+1: #collected block
                            obstacle = False
                            goinghome = True
                            path = np.insert(path, i+2, home, axis=0)
                            #region
                            # print(type(last_known_point[0]))
                            # print(type(cartesian_bearing))
                            # path.insert(i+2, [last_known_point[0] + 0.1 * np.cos(cartesian_bearing * deg2rad), last_known_point[1] + 0.1 * np.sin(cartesian_bearing * deg2rad)])
                            # print(i)
                            # print(path.size)
                            # print(last_known_point)
                            # path = np.insert(path, i+2, 
                            #                 [last_known_point[0] + 0.1 * np.cos(cartesian_bearing * deg2rad),
                            #                  last_known_point[1] + 0.1 * np.sin(cartesian_bearing * deg2rad)],
                            #                  axis=0)
                            # if path_turns == 0:
                            #     # path.insert(i+3, home)
                            #     path = np.insert(path, i+3, home, axis=0)
                            # else:
                            #     # path.insert(i+3, [last_known_point[0], 1])
                            #     # path.insert(i+4, home)
                            #     path = np.insert(path, i+3, [last_known_point[0], 1], axis=0)
                            #     path = np.insert(path, i+4, home, axis=0)
                            #endregion

                    elif colour == other_robot_colour: #implement avoidance function
                        print('nah screw you')
                        #send gps coordinates to other robot
                        if blockcoords_sent == False:
                            message_block = [1, *block_coords] # 0 - robot's coordinates, 1 - block coordinates 
                            message_block = struct.pack("3f", *message_block)
                            emitter.send(message_block)
                            blockcoords_sent = True
                            # reset blockcoords_sent after avoiding obstacle
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
    else:
        print('too close')
        leftSpeed = 0.0
        rightSpeed = 0.0
    
    if robot.getTime() >= timeout:
        print("Out-of-time !")
        current_coordinates = getCoordinates(gps)
        # previous_coordinates = current_coordinates
        
        if robot.getTime() == timeout:
            previous_coordinates = current_coordinates
            path = np.array([previous_coordinates, current_coordinates, home])
            i = 0
            # path = np.append(path, [current_coordinates], axis=0)
        
        # rotateTo(previous_coordinates, current_coordinates, desired_coordinates, current_bearing, alignment)

        moveTo(previous_coordinates, current_coordinates, home, current_bearing, i)
        # path = np.append(path, [home], axis=0)
        unloading   = False
        obstacle    = False
        goinghome   = True
        
        if atHome:
            print("atHome")
            leftSpeed  = 0.0
            rightSpeed = 0.0
    
    moveTo(previous_coordinates, current_coordinates, home, current_bearing, i)
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    previous_coordinates = current_coordinates
