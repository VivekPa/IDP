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

# camera = robot.getDevice('camera')
# camera.enable(TIME_STEP)

camera_left = robot.getDevice('camera_left')
camera_left.enable(TIME_STEP)

camera_right = robot.getDevice('camera_right')
camera_right.enable(TIME_STEP)

#endregion

""""""

while robot.step(TIME_STEP) != -1:
    # get current device values
    current_coordinates = getCoordinates(gps)
    current_bearing = getBearing(compass)
    ds_1_value = ds_left.getValue()
    ds_2_value = ds_right.getValue()

    if a == len(path)-2:
        # print('R reached the end')
        if other_robot_done == True:
            if started_collecting == False:
                #adds list of blocks detected by other robot to the path only after blue robot finishes it sweep
                print('add list of blocks to path')
                path = np.append(path, np.array(list_of_blocks), axis = 0)
                started_collecting = True
                
            else: #finished collecting (when started collecting == True and reaches end of path)
                #sends message to blue robot after red robot finishes collecting its blocks
                print('sent end of collection message')
                message = [2,0,0] # first digit: 0 - robot's coordinates, 1 - block coordinates , 2- done sweeping/collecting #2nd and 3rd digit: dummy values
                message = struct.pack("3f", *message)
                emitter.send(message)
                path = np.vstack([path, current_coordinates])
                other_robot_done = False
                #started_collecting = False
                robot_final_done = True
        else:
            #to prevent it from reaching end of path and timing out
            a-=1
            
        # print('going home')
        # obstacle = False
        # goinghome = True
        # destination = standby
        # path = get_total_path(current_coordinates,ox,oy,destination,path,a,show_animation = False)
        # leftSpeed, rightSpeed = 0,0

    #message_robot = [0, *current_coordinates] # 0 - robot's coordinates, 1 - block coordinates, 2-done sweeping/collecting
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
            list_of_blocks.append(list(message[1:]))
            #path = np.append(path, np.array([message[1:]]), axis = 0)
        elif message[0] == 2: #change status of other robot if it receives a 2
            other_robot_done = True

        # print('Blue robot location:', other_robot_coordinates)
        receiver.nextPacket() #deletes the head packet
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
    if num_blocks_collected < 4:
        if distance_btw_robots > 2*0.22:
            getting_away = False
            # detect obstacles
            right_obstacle = ds_1_value < 1000.0
            left_obstacle = ds_2_value < 1000.0
            #call obstacle_check only if obstacle outside of while loop and unloading is false
            #potentially changes obstacle to True if it detects a block
            if obstacle == False and unloading == False:
                obstacle1 = False
                obstacle2 = False
                if right_obstacle == True:
                    # obstacle_coords = find_obstacle_coords(ds_left, gps,compass)
                    block_coords1, obstacle1 = obstacle_check(ds_left, gps, compass, obstacle, other_colour_blocks, indetermined_obs_blocks)
                    #last_known_point = getCoordinates(gps)
                else:
                    pass
                if left_obstacle == True:
                    block_coords2, obstacle2 = obstacle_check(ds_right, gps, compass, obstacle, other_colour_blocks, indetermined_obs_blocks)
                    #last_known_point = getCoordinates(gps)
                    # print(last_known_point)
                else:
                    pass
                if obstacle1 == True or obstacle2 == True:
                    obstacle = True
                    if obstacle1:
                        block_coords = block_coords1
                    elif obstacle2:
                        block_coords = block_coords2
            else:
                pass
            
            desired_coordinates = path[a+2]
            if list(desired_coordinates) in og_path:
                # print('R on og path', list(desired_coordinates))
                check_distance = 0.2
                block_in_the_way = check_next_point(current_coordinates, desired_coordinates, check_distance, other_colour_blocks)
                if block_in_the_way:
                    print('shld plan new path')
                    for obs_coords in other_colour_blocks:
                        while np.linalg.norm(np.array(obs_coords) - np.array(path[a+2])) < 0.2:
                            a+=1

                    destination = path[a+2]
                    path = get_total_path(current_coordinates,ox,oy,destination,path,a,show_animation = False)
                    leftSpeed, rightSpeed = 0,0
                    desired_coordinates = path[a+2]

            if unloading == False:
                #obstacle Boolean here might be different from the obstacle boolean at the start of this loop due to the previous if statement
                if obstacle == True and goinghome == False:
                    alignment = False
                    leftSpeed, rightSpeed, alignment = rotateTo(previous_coordinates, current_coordinates, block_coords, current_bearing, alignment)
                    if alignment == True:
                        if colour_determined == False:
                            colour = getRGB(camera_left, camera_right)
                            #colour = getRGB(camera)
                            alignment = False #switch alignment back to false
                            colour_determined = True
                            print('R Colour (0 - Red, 1-Green, 2- Blue, None- 1 each):', colour)
                            if colour == robot_colour: #implement collection function
                                last_known_location = path[a+2]
                                print('R correct colour, collect it')
                                collection_coords = calc_collection_coords(block_coords, current_bearing)
                                leftSpeed, rightSpeed, b = moveTo(previous_coordinates, current_coordinates, collection_coords, current_bearing, a)
                                if b == a+1: #collected block
                                    print('going home')
                                    obstacle = False
                                    goinghome = True
                                    destination = home
                                    path = get_total_path(current_coordinates,ox,oy,destination,path,a,show_animation = False)
                                    leftSpeed, rightSpeed = 0,0

                            elif colour == other_robot_colour: #implement avoidance function
                                print('R wrong colour, avoiding')
                                #send gps coordinates to other robot
                                if blockcoords_sent == False:
                                    message_block = [1, *block_coords] # 0 - robot's coordinates, 1 - block coordinates 
                                    message_block = struct.pack("3f", *message_block)
                                    emitter.send(message_block)
                                    blockcoords_sent = True
                                    # reset blockcoords_sent after avoiding obstacle
                                
                                other_colour_blocks.append(list(block_coords))
                                ox.append(round(block_coords[1]*100))
                                oy.append(round(block_coords[0]*100))
                                while np.linalg.norm(np.array(block_coords) - np.array(path[a+2])) < 0.2:
                                    if a == len(path)-3: #this bit is messy and not exactly correct..just ignore it
                                        if started_collecting == True:
                                            message = [2,0,0] # first digit: 0 - robot's coordinates, 1 - block coordinates , 2- done sweeping/collecting #2nd and 3rd digit: dummy values
                                            message = struct.pack("3f", *message)
                                            emitter.send(message)
                                            reverse_coords = calc_reverse_coords(current_coordinates, current_bearing)
                                            path = np.vstack([path, reverse_coords])
                                            a=+1
                                            # path = np.vstack([path, current_coordinates])
                                            other_robot_done = False
                                            started_collecting = False
                                            robot_final_done = True
                                            break
                                        else:
                                            reverse_coords = calc_reverse_coords(current_coordinates, current_bearing)
                                            path = np.vstack([path, reverse_coords])
                                            a+=1
                                            break
                                    else:
                                        a+=1

                                destination = path[a+2]
                                path = get_total_path(current_coordinates,ox,oy,destination,path,a, show_animation=False)
                                colour_determined = False
                                obstacle = False
                                blockcoords_sent = False
                                leftSpeed, rightSpeed = 0,0
                            elif colour == 1:
                                print('R green')
                                leftSpeed, rightSpeed, alignment = rotateTo(previous_coordinates, current_coordinates, block_coords, current_bearing-5, alignment)
                                colour_determined = False
                            elif colour == None:
                                print('R cant determine')
                                if blockcoords_sent == False:
                                    message_block = [1, *block_coords] # 0 - robot's coordinates, 1 - block coordinates 
                                    message_block = struct.pack("3f", *message_block)
                                    emitter.send(message_block)
                                    blockcoords_sent = True
                                    # reset blockcoords_sent after avoiding obstacle
                                
                                other_colour_blocks.append(list(block_coords))
                                ox.append(round(block_coords[1]*100))
                                oy.append(round(block_coords[0]*100))

                                while np.linalg.norm(np.array(block_coords) - np.array(path[a+2])) < 0.2:
                                    if a == len(path)-3: #this bit is messy and not exactly correct..just ignore it
                                        if started_collecting == True:
                                            message = [2,0,0] # first digit: 0 - robot's coordinates, 1 - block coordinates , 2- done sweeping/collecting #2nd and 3rd digit: dummy values
                                            message = struct.pack("3f", *message)
                                            emitter.send(message)
                                            reverse_coords = calc_reverse_coords(current_coordinates, current_bearing)
                                            path = np.vstack([path, reverse_coords])
                                            a+=1
                                            # path = np.vstack([path, current_coordinates])
                                            other_robot_done = False
                                            started_collecting = False
                                            robot_final_done = True
                                            break
                                        else:
                                            reverse_coords = calc_reverse_coords(current_coordinates, current_bearing)
                                            path = np.vstack([path, reverse_coords])
                                            a+=1
                                            break
                                    else:
                                        a+=1

                                destination = path[a+2]
                                path = get_total_path(current_coordinates,ox,oy,destination,path,a, show_animation=False)
                                colour_determined = False
                                obstacle = False
                                blockcoords_sent = False
                                leftSpeed, rightSpeed = 0,0

                        else: #colour already determined
                            if colour == robot_colour: #implement collection function
                                collection_coords = calc_collection_coords(block_coords, current_bearing)
                                leftSpeed, rightSpeed, b = moveTo(previous_coordinates, current_coordinates, collection_coords, current_bearing, a)
                                if b == a+1: #collected block
                                    print('going home')
                                    colour_determined = False
                                    obstacle = False
                                    goinghome = True
                                    destination = home
                                    path = get_total_path(current_coordinates,ox,oy,destination,path,a,show_animation = False)
                                    leftSpeed, rightSpeed = 0,0

                            elif colour == other_robot_colour: #implement avoidance function
                                pass
                            elif colour == 1:
                                pass
                            elif colour == None:
                                pass

                elif obstacle == True and goinghome == True:
                    print('R trying to avoid')
                    #implement avoidance function
                    indetermined_obs_blocks.append(list(block_coords))
                    ox.append(round(block_coords[1]*100))
                    oy.append(round(block_coords[0]*100))
                    while np.linalg.norm(np.array(block_coords) - np.array(path[a+2])) < 0.2:
                        # a+=1
                        if a == len(path)-3: #same here..this bit is messy and not exactly correct..just ignore it
                            if started_collecting == True:
                                message = [2,0,0] # first digit: 0 - robot's coordinates, 1 - block coordinates , 2- done sweeping/collecting #2nd and 3rd digit: dummy values
                                message = struct.pack("3f", *message)
                                emitter.send(message)
                                reverse_coords = calc_reverse_coords(current_coordinates, current_bearing)
                                path = np.vstack([path, reverse_coords])
                                a+=1
                                # path = np.vstack([path, current_coordinates])
                                other_robot_done = False
                                started_collecting = False
                                robot_final_done = True
                                break
                            else:
                                reverse_coords = calc_reverse_coords(current_coordinates, current_bearing)
                                path = np.vstack([path, reverse_coords])
                                a+=1
                                break
                        else:
                            a+=1

                    destination = path[a+2]
                    path = get_total_path(current_coordinates,ox,oy,destination,path,a,show_animation = False)
                    leftSpeed, rightSpeed = 0,0
                
                    obstacle = False
                else:
                    leftSpeed, rightSpeed, a = moveTo(previous_coordinates, current_coordinates, desired_coordinates, current_bearing, a)

            elif unloading == True:
                leftSpeed, rightSpeed, b = reverseTo(previous_coordinates, current_coordinates, reverse_coords, a)
                if b == a + 1:
                    a += 1
                    num_blocks_collected +=1
                    print(num_blocks_collected)
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
                    path = get_total_path(current_coordinates,ox,oy,destination,path,a, show_animation = False)
                    
        elif distance_btw_robots <= 2*0.22 and getting_away == False:
            print('R too close')
            if robot_final_done == True: #if the red robot finished collecting and the blue robot is collecting, just let the blue robot avoid it
                leftSpeed, rightSpeed = 0,0
            else:
                getting_away = True

                #create a square of obstacles for the robot
                for i in range(round(other_robot_coordinates[1]*100)-20,round(other_robot_coordinates[1]*100)+21,2): 
                    ox.append(i)
                    oy.append(round(other_robot_coordinates[0]*100)-20)
                    ox.append(i)
                    oy.append(round(other_robot_coordinates[0]*100)+20)

                for i in range(round(other_robot_coordinates[0]*100)-20,round(other_robot_coordinates[0]*100)+21,2): 
                    oy.append(i)
                    ox.append(round(other_robot_coordinates[1]*100)-20)
                    oy.append(i)
                    ox.append(round(other_robot_coordinates[1]*100)+20)

                while np.linalg.norm(np.array(other_robot_coordinates) - np.array(path[a+2])) < 0.41: #can change this distance
                    # if list(path[a+2]) in list_of_blocks:
                    #     path = np.append(path, np.array([path[a+2]]), axis = 0) # append that point to the back
                    if a == len(path)-3:
                        if started_collecting == True:
                            print('asked blue robot to start moving')
                            message = [2,0,0] # first digit: 0 - robot's coordinates, 1 - block coordinates , 2- done sweeping/collecting #2nd and 3rd digit: dummy values
                            message = struct.pack("3f", *message)
                            emitter.send(message)
                            reverse_coords = calc_reverse_coords(current_coordinates, current_bearing)
                            path = np.vstack([path, reverse_coords])
                            a+=1
                            # path = np.vstack([path, current_coordinates])
                            other_robot_done = False
                            started_collecting = False
                            robot_final_done = True
                            break
                        else:
                            reverse_coords = calc_reverse_coords(current_coordinates, current_bearing)
                            path = np.vstack([path, reverse_coords])
                            a+=1
                            # path = np.vstack([path, current_coordinates])
                            break
                    else:
                        a+=1

                    

                destination = path[a+2]
                path = get_total_path(current_coordinates,ox,oy,destination,path,a,show_animation = True)
                #get_total_path(current_coordinates,ox,oy,destination,path,a,show_animation=True,robot_radius=19.5) #can change robot radius so that there is a larger buffer but risk no paths
                desired_coordinates = path[a+2]
                leftSpeed, rightSpeed, a = moveTo(previous_coordinates, current_coordinates, desired_coordinates, current_bearing, a)
                for i in range(84): #delete robot's obstacles
                    ox.pop(-1)
                    oy.pop(-1)
        
        else:
            desired_coordinates = path[a+2]
            leftSpeed, rightSpeed, a = moveTo(previous_coordinates, current_coordinates, desired_coordinates, current_bearing, a)
    else:
        leftSpeed, rightSpeed = 0,0
        if end_of_collection_message_sent == False:
            end_of_collection_message_sent = True
            print('all blocks collected')
            print('sent end of collection message')
            message = [2,0,0] # first digit: 0 - robot's coordinates, 1 - block coordinates , 2- done sweeping/collecting #2nd and 3rd digit: dummy values
            message = struct.pack("3f", *message)
            emitter.send(message)
    if robot.getTime() >= timeout:
        print("R Out-of-time !")
        current_coordinates = getCoordinates(gps)
        # previous_coordinates = current_coordinates
        
        if robot.getTime() == timeout:
            previous_coordinates = current_coordinates
            path = np.array([previous_coordinates, current_coordinates, home])
            a = 0
        desired_coordinates = path[a+2]
        leftSpeed, rightSpeed, a = moveTo(previous_coordinates, current_coordinates, desired_coordinates, current_bearing, a)
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
