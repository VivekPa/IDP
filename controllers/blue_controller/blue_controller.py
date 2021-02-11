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

""""""
while robot.step(TIME_STEP) != -1:
    if i == len(path)-2:
        #print('B reached the end')
        i -= 1
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
    while receiver.getQueueLength() > 0:
        message = receiver.getData()
        message = np.array(list(struct.unpack("3f",message)))
        if round(message[0]) == 0.0:
            other_robot_coordinates = message[1:]
        elif message[0] == 1.0 :
            #path = np.append(path, np.array([message[1:]]), axis = 0)
            list_of_blocks = np.append(list_of_blocks, np.array([message[1:]]), axis = 0)

        # print('Red robot location:', other_robot_coordinates)
        receiver.nextPacket() #deletes the head packet
    
    # if receiver.getQueueLength() == 0:
    #     print('no message')
    distance_btw_robots = np.linalg.norm(np.array(current_coordinates) - np.array(other_robot_coordinates))
    # print('distance', distance_btw_robots)
    if distance_btw_robots > 2*0.25:
        # detect obstacles
        right_obstacle = ds_1_value < 1000.0
        left_obstacle = ds_2_value < 1000.0

        # Check if the robot is near home
        if np.linalg.norm(np.array(current_coordinates) - np.array(home)) < 0.1:
            atHome = True
        else:
            atHome = False
            
        if np.linalg.norm(np.array(current_coordinates) - np.array(home)) < 0.3:
            nearHome = True
        else:
            nearHome = False
        # print('athome', atHome)
        """ Unloading """
        #region
        if atHome and robot.getTime() > 8:  # If the robot is near home after 'return_to_home()'
            unloading = True                # Initiate unloading procedure
            reverse_coords = calc_reverse_coords(current_coordinates, current_bearing)
        # print('test',i,i_whennewptsadded)
        if obstacle_avoided == False:
            if i == (i_whennewptsadded + 3):
                obstacle_avoided = True
        #call obstacle_check only if obstacle outside of while loop and unloading is false
        #potentially changes obstacle to True if it detects a block
        # if obstacle == False and unloading == False and obstacle_avoided == True:
        if obstacle == False and nearHome == False and obstacle_avoided == True:
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

        # calculating distance between the desired coordinate and current coordinate
        desired_coordinates = path[i+2]

        # #check if robot has made a turn
        # coordinates = path[i+1]
        # print(path_turns, coordinates)
        # if coordinates[0] == turnpoints[0][0] and coordinates[1] == turnpoints[0][1]:
        #     path_turns += 1
        #     print(coordinates,turnpoints)
        #     turnpoints = np.delete(turnpoints,0, axis=0)
        #     print(turnpoints)
        #     print('turned')

        if unloading == False:
            #obstacle Boolean here might be different from the obstacle boolean at the start of this loop due to the previous if statement
            if obstacle == True and goinghome == False:
                alignment = False
                leftSpeed, rightSpeed, alignment = rotateTo(previous_coordinates, current_coordinates, block_coords, current_bearing, alignment)
                if alignment == True:
                    print('B aligned')
                    colour = getRGB(camera_left, camera_right)
                    alignment = False #switch alignment back to false
                    print('B Colour:', colour)
                    if colour == robot_colour or colour == None or colour == 1: #implement collection function
                        print('B Correct colour, collect it')
                        leftSpeed, rightSpeed, j = moveTo(previous_coordinates, current_coordinates, block_coords, current_bearing, i)
                        if j == i+1: #collected block
                            obstacle = False
                            goinghome = True
                            path = np.insert(path, i+2, home, axis=0)
                            path = np.insert(path, i+3, last_known_point, axis=0)
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

                    elif colour == other_robot_colour: #implement avoidance function
                        print('B wrong colour, avoiding')
                        #send gps coordinates to other robot
                        if blockcoords_sent == False:
                            message_block = [1, *block_coords] # 0 - robot's coordinates, 1 - block coordinates 
                            message_block = struct.pack("3f", *message_block)
                            emitter.send(message_block)
                            blockcoords_sent = True
                            # reset blockcoords_sent after avoiding obstacle
                        bypass_points = avoid_obstacle(current_coordinates, block_coords, current_bearing)
                        print(bypass_points)
                        for j in range(bypass_points.shape[0]):
                            path = np.insert(path, i+2+j, bypass_points[j] , axis=0)
                        
                        i_whennewptsadded = i
                        obstacle = False #change obstacle back to False after collecting the block
                        obstacle_avoided = False
                        blockcoords_sent = False

                    # elif colour == None:
                    #     print('cant determine')
                    #     # leftSpeed  = 0
                    #     # rightSpeed = 0

                    
                    #print(obstacle)
                    # leftMotor.setVelocity(leftSpeed)
                    # rightMotor.setVelocity(rightSpeed)
                    # print('trying to break')
                    # break
            elif obstacle == True and goinghome == True:
                print('B trying to avoid')
                bypass_points = avoid_obstacle(current_coordinates, block_coords, current_bearing)         
                for j in range(bypass_points.shape[0]):
                    path = np.insert(path, i+2+j, bypass_points[j] , axis=0)
                        
                i_whennewptsadded = i
                obstacle = False #change obstacle back to False after collecting the block
                obstacle_avoided = False
                #implement avoidance function
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
        print('B too close')
        # if path[i+2][0] != home[0] and path[i+2][1] != home[1]:
        #     print('try going home first')
        #     path = np.insert(path, i+2, home, axis=0)
        #     obstacle = False
        #     goinghome = True
        # else:
        #     print('already going home')
        # # leftSpeed = 0
        # # rightSpeed = 0
        # desired_coordinates = path[i+2]
        # leftSpeed, rightSpeed, i = moveTo(previous_coordinates, current_coordinates, desired_coordinates, current_bearing, i)

    if robot.getTime() >= timeout:
        print("Out-of-time!")
        current_coordinates = getCoordinates(gps)
        # previous_coordinates = current_coordinates
        
        if robot.getTime() == timeout:
            path = np.array([previous_coordinates, current_coordinates, home])
            i = 0
            obstacle = False
            goinghome = True
            # path = np.insert(path, i+2, home, axis=0)
            # path = np.append(path, [current_coordinates], axis=0)
        
        # rotateTo(previous_coordinates, current_coordinates, desired_coordinates, current_bearing, alignment)

        # leftSpeed, rightSpeed, i = moveTo(previous_coordinates, current_coordinates, home, current_bearing, i)
        # # path = np.append(path, [home], axis=0)
        # unloading   = False
        # obstacle    = False
        # goinghome   = True
        
        if atHome:
            print("R atHome, done for the day")
            leftSpeed  = 0.0
            rightSpeed = 0.0

    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    previous_coordinates = current_coordinates
