from controller import Robot, Motor, DistanceSensor, GPS, Emitter, Receiver, Camera
import math
import numpy as np
import struct

from utils.variables import *
from utils.obstacle_detection import *
from utils.sensors_api import *
from utils.motion_api import *
from utils.astarplanner import *
from utils.path_planning import *


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
    if a == len(path)-2:
        print('reached the end')

    # get current device values
    current_coordinates = getCoordinates(gps)
    current_bearing = getBearing(compass)
    ds_1_value = ds_left.getValue()
    ds_2_value = ds_right.getValue()

    # #message_robot = [0, *current_coordinates] # 0 - robot's coordinates, 1 - block coordinates
    # message_robot = [0]                                         # Select message type as robot coordinates
    # message_robot.extend(current_coordinates)                   # Send coordinates
    # message_robot = struct.pack("3f",   message_robot[0],       # Pack message type
    #                                     message_robot[1],       # Pack x coordinate
    #                                     message_robot[2])       # Pack z coordinate

    # emitter.send(message_robot)

    # #region
    # #receive other robot's coordinates
    # #print('Receiver Queue length:'  , receiver.getQueueLength())
    # while receiver.getQueueLength() > 0:
    #     message = receiver.getData()
    #     message = np.array(list(struct.unpack("3f",message)))
    #     if message[0] == 0:
    #         other_robot_coordinates = message[1:]
    #     elif message[0] == 1:
    #         np.append(list_of_blocks, message[1:])

    #     # print('Blue robot location:', other_robot_coordinates)
    #     receiver.nextPacket() #deletes the head packet
    
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
        print(obstacle,left_obstacle,right_obstacle)
        #call obstacle_check only if obstacle outside of while loop and unloading is false
        #potentially changes obstacle to True if it detects a block
        if obstacle == False and unloading == False:
            if right_obstacle == True:
                # obstacle_coords = find_obstacle_coords(ds_left, gps,compass)
                block_coords, obstacle = obstacle_check(ds_left, gps, compass, obstacle, other_colour_blocks, indetermined_obs_blocks)
                #last_known_point = getCoordinates(gps)
            else:
                pass
            if left_obstacle == True:
                block_coords, obstacle = obstacle_check(ds_right, gps, compass, obstacle, other_colour_blocks, indetermined_obs_blocks)
                #last_known_point = getCoordinates(gps)
                # print(last_known_point)
            else:
                pass
        else:
            pass
        
        desired_coordinates = path[a+2]

        if unloading == False:
            #obstacle Boolean here might be different from the obstacle boolean at the start of this loop due to the previous if statement
            if obstacle == True and goinghome == False:
                alignment = False
                leftSpeed, rightSpeed, alignment = rotateTo(previous_coordinates, current_coordinates, block_coords, current_bearing, alignment)
                if alignment == True:
                    colour = getRGB(camera_left, camera_right)
                    alignment = False #switch alignment back to false
                    print('R Colour (0 - Red, 1-Green, 2- Blue, None- 1 each):', colour)
                    if colour == robot_colour: #implement collection function
                        last_known_location = path[a+2]
                        print('R correct colour, collect it')
                        leftSpeed, rightSpeed, b = moveTo(previous_coordinates, current_coordinates, block_coords, current_bearing, a)
                        if b == a+1: #collected block
                            obstacle = False
                            goinghome = True
                            destination = home
                            path = get_total_path(current_coordinates,ox,oy,destination,path,a)
                            leftSpeed, rightSpeed = 0,0

                    elif colour == other_robot_colour or colour == 1: #implement avoidance function
                        print('R wrong colour, avoiding')
                        #send gps coordinates to other robot
                        # if blockcoords_sent == False:
                        #     message_block = [1, *block_coords] # 0 - robot's coordinates, 1 - block coordinates 
                        #     message_block = struct.pack("3f", *message_block)
                        #     emitter.send(message_block)
                        #     blockcoords_sent = True
                        #     # reset blockcoords_sent after avoiding obstacle
                        
                        other_colour_blocks.append(list(block_coords))
                        ox.append(round(block_coords[1]*100))
                        oy.append(round(block_coords[0]*100))

                        while np.linalg.norm(np.array(block_coords) - np.array(path[a+2])) < 0.2:
                            a+=1

                        destination = path[a+2]
                        path = get_total_path(current_coordinates,ox,oy,destination,path,a)
                    
                        obstacle = False
                        blockcoords_sent = False
                        leftSpeed, rightSpeed = 0,0
                        print('path', path[a+2:])
                    
                    elif colour == None:
                        print('R cant determine')
                        leftSpeed  = 0
                        rightSpeed = 0

            elif obstacle == True and goinghome == True:
                print('trying to avoid')
                #implement avoidance function
                indetermined_obs_blocks.append(list(block_coords))
                print(indetermined_obs_blocks)
                ox.append(round(block_coords[1]*100))
                oy.append(round(block_coords[0]*100))
                while np.linalg.norm(np.array(block_coords) - np.array(path[a+2])) < 0.2:
                    a+=1

                destination = path[a+2]
                path = get_total_path(current_coordinates,ox,oy,destination,path,a)
                leftSpeed, rightSpeed = 0,0
            
                obstacle = False
            else:
                leftSpeed, rightSpeed, a = moveTo(previous_coordinates, current_coordinates, desired_coordinates, current_bearing, a)

        elif unloading == True:
            leftSpeed, rightSpeed, b = reverseTo(previous_coordinates, current_coordinates, reverse_coords, a)
            if b == a + 1:
                a += 1
                #finish unloading, reset all state variables
                goinghome = False
                unloading = False
                obstacle = False
                while np.linalg.norm(np.array(home) - np.array(path[a+2])) < 0.1:
                    a+=1
                for i in range(len(indetermined_obs_blocks)): #remove those indetermined_obs_blocks from obstacle list
                    ox.pop(-1)
                    oy.pop(-1)
                indetermined_obs_blocks = [] #clear indetermined_obs_blocks
                destination = path[a+2]
                path = get_total_path(current_coordinates,ox,oy,destination,path,a)
                
    else:
        print('R too close')
        leftSpeed = 0.0
        rightSpeed = 0.0
    
    if robot.getTime() >= timeout:
        print("R Out-of-time !")
        current_coordinates = getCoordinates(gps)
        # previous_coordinates = current_coordinates
        
        if robot.getTime() == timeout:
            previous_coordinates = current_coordinates
            path = np.array([previous_coordinates, current_coordinates, home])
            a = 0
            # path = np.append(path, [current_coordinates], axis=0)
        
        # rotateTo(previous_coordinates, current_coordinates, desired_coordinates, current_bearing, alignment)

        moveTo(previous_coordinates, current_coordinates, home, current_bearing, a)
        # path = np.append(path, [home], axis=0)
        unloading   = False
        obstacle    = False
        goinghome   = True
        
        if atHome:
            print("R atHome")
            leftSpeed  = 0.0
            rightSpeed = 0.0
    
    moveTo(previous_coordinates, current_coordinates, home, current_bearing, a)
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    previous_coordinates = current_coordinates
