"""preplannedpath controller."""

# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, DistanceSensor, GPS, Emitter, Receiver, Camera
from subdir.functions import *
import math
import struct #to convert native python data types into a string of bytes and vice versa
import numpy as np

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
TIME_STEP = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:

""" Initialise sensors """

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

""" Define Waypoints and Home """
red_base = [1,0,1]
blue_base = [1,0,-1]
home = blue_base
other_robot_coordinates = red_base
path = [home,home,[1,0,-0.2],[0,0,-0.2],[-1,0,-0.2],[-1,0,-0.6],[0,0,-0.6],[0.6,0,-0.6],[0.6,0,-1],[0,0,-1],[-1,0,-1]] # always duplicate first point
#path = [[1,1], [1,1],[0,1],[1,1],[-1,0],[0,0],[-1,-1],[0,0]] # always duplicate first point
atHome      = True               # Home region

""" State Variables """
unloading   = False              # Unloading state
obstacle = False                 # Obstacle detection state
goinghome = False

i = 0                            # Path index

previous_coordinates = path[0]
robot_colour = 2                 # 0 - red, 1 - green, 2- blue
other_robot_colour = 0

deg2rad = 3.14159/180

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

    Note: Angles given are taken from the z-axis of the robot.
    """
    if ds == 'ds_1':
        ds_distance = 0.1089770618
        ds_angle = 60
        ds_disp_angle = -42.76882539
    elif ds == 'ds_2':
        ds_distance = 0.1089770618
        ds_angle = -60
        ds_disp_angle = 42.76882539
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
        ds_value = 0.0003 * ds_left.getValue() + 0.011
    elif ds == 'ds_2':
        ds_value = 0.0003 * ds_right.getValue() + 0.011
    else:
        print('distance sensor not found')
    # print(ds_value)
    return ds_value

def find_obstacle_coords(ds):
    """
    A function which returns the coordinates of the incident point on the
    surface which the distance sensor is detecting.

    Arguments: ds (a string which denotes the distance sensor which detected an
    obstacle)
    """
    #make a rough estimate of coordinates of the point on a surface it 'sees
    #retrieve reading
    ds_reading = ds_read(ds)
    #print(ds_reading)
    #retrieve attributes
    ds_attributes = get_attributes(ds)
    ds_distance = ds_attributes[0]
    # print(ds_distance)
    #retrieve gps and bearing
    gps_reading = gps.getValues()
    bearing = getBearing(compass.getValues())
    #find absolute angle of detector. remember to convert to radians!
    ds_absolute_angle = (bearing + ds_attributes[1]) * (deg2rad)
    ds_absolute_disp_angle = (bearing + ds_attributes[2]) * (deg2rad)
    #print(ds_absolute_angle, ds_absolute_disp_angle)
    #find coordinates.
    x_coord = (ds_reading * np.cos(ds_absolute_angle)) + (ds_distance * np.cos(ds_absolute_disp_angle)) + gps_reading[0]
    z_coord = (ds_reading * np.sin(ds_absolute_angle)) + (ds_distance * np.sin(ds_absolute_disp_angle)) + gps_reading[2]
    return x_coord, z_coord

def find_block_coords(prelim_coords, bearing, ds):
    """
    A function which returns the coordinates of the centre of a block.

    Arguments: prelim_coords (the incident point's coordinates), bearing(the
    robot's bearing in degrees), ds (the distance sensor which detected an
    obstacle)
    """
    #find out the cartesian direction of the robot.
    cartesian_bearing = bearing_round(bearing)
    #use diagonal distance from corner to centre of block
    block_diagonal = 0.03535533906
    #the corner detected will depend on the cartesian direction of the robot and
    #the distance sensor which detected the block (as they point in opposing
    #directions)
    if ds == 'ds_1':
        #find angle of corner to centre. remember to convert to radians!
        diagonal_absolute_angle = (bearing_round(bearing) - 45) * (deg2rad)
        x_block = prelim_coords[0] + block_diagonal * np.cos(diagonal_absolute_angle)
        z_block = prelim_coords[1] + block_diagonal * np.sin(diagonal_absolute_angle)
    elif ds == 'ds_2':
        diagonal_absolute_angle = (bearing_round(bearing) + 45) * (deg2rad)
        x_block = prelim_coords[0] + block_diagonal * np.cos(diagonal_absolute_angle)
        z_block = prelim_coords[1] + block_diagonal * np.sin(diagonal_absolute_angle)
    else:
        print('distance sensor not found')
    block_coords = [x_block, 0, z_block]
    print('block_coords:', block_coords)
    return block_coords

def obstacle_check(ds,obstacle):
    """
    A function called if the distance sensors detect an object within the sweep lane. It determines whether the object
    is a block or a wall, calling the reciprocating_sweep function if it is a block.

    Arguments: ds (a string which denotes the distance sensor which detected an obstacle)
    Returns: block_coords, obstacle (Boolean)
    """
    x_prelim, z_prelim = find_obstacle_coords(ds)
    prelim_coords = [x_prelim, z_prelim]
    # print(prelim_coords, gps.getValues())
    #check if the object is a wall, by comparing with the lines x = 1.2/-1.2,
    #z = 1.2/-1.2
    wall_coord = 1.2
    wall_tolerance = 0.01
    lower_wall = wall_coord - wall_tolerance
    upper_wall = wall_coord + wall_tolerance
    if lower_wall <= abs(x_prelim) <= upper_wall or lower_wall <= abs(z_prelim) <= upper_wall:
        # print('all okay! Just a wall')
        obstacle = False
        block_coords = None
        pass
    else:
        # print('thats no moon!')
        obstacle = True
        block_coords = find_block_coords(prelim_coords, getBearing(compass.getValues()), ds)
    
    return block_coords, obstacle
"""
def reciprocating_sweep(ds):
"""
    #A function which the robot uses to find the edge of a detected block. It returns the coordinates of the block.

    #Arguments: ds (a string which denotes the distance sensor which detected an obstacle)
"""
    #Retrieve attributes
    ds_attributes = get_attributes(ds)
    ds_distance = ds_attributes[0]
    ds_absolute_angle = getBearing(compass.getValues()) + ds_attributes[1]
    ds_absolute_disp_angle = getBearing(compass.getValues()) + ds_attributes[2]
    #move back 80mm
    #find coordinates 80mm behind
    back_coords = [(gps.getValues()[0] - 0.08 * np.sin(getBearing(compass.getValues()))), (gps.getValues()[2] - 0.08 * np.cos(getBearing(compass.getValues())))]
    moveto(get_gps_xz(gps.getValues()), get_gps_xz(gps.getValues()), back_coords(), getBearing(), i)
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
"""

def go_to_block(block_coords):
    """
    This function give the desired coords to a position 5cm behind a block when
    found in order to determine its colour. Empty for now.

    Arguments: block_coords (an array of the block coordinates found in
    obstacle_check)
    """


def stop():
    """
    This function stops the robot.
    """
    leftMotor.setVelocity(0.0)
    rightMotor.setVelocity(0.0)

"""
End of search functions
"""

def return_to_home():
    """
    This function returns the robot to home.
    """
    path.insert(i+2,home)

def unload():
    """
    This function allows the collected block to be unloaded by:
        * Moving the robot back by __ distance
        * rotate to next waypoint
    """
    leftSpeed  = -0.5 * MAX_SPEED
    rightSpeed = -0.5 * MAX_SPEED

    return leftSpeed, rightSpeed

def getRGB():
    """
    Returns: integer cooresponding to colour with largest pixel value as well individual pixel values
        0: red
        1: green
        2: blue
    """
    image_left = camera_left.getImageArray()
    image_right = camera_right.getImageArray()
    RGB_left = image_left[0][0]
    RGB_right = image_right[0][0]
    print(RGB_left)
    print(RGB_right)
    colour_left = RGB_left.index(max(RGB_left))
    colour_right = RGB_right.index(max(RGB_right))
    if colour_left == colour_right:
        colour = colour_left
        if colour == 1:
            print('error! both green')
    elif colour_left != 1 and colour_right != 1:
        print('error! one red one blue')
        colour = None
    else:
        if colour_left == 1:
            colour = colour_right
        else:
            colour = colour_left


    # red   = RGB[0]
    # green = RGB[1]
    # blue  = RGB[2]

    return colour  #, red, green, blue


# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    if i == len(path)-2:
        print('reached the end')
    
    # get current device values
    current_coordinates = gps.getValues()
    north = compass.getValues()
    current_bearing = getBearing(north)
    ds_1_value = ds_left.getValue()
    ds_2_value = ds_right.getValue()

    #send gps coordinates to other robot
    message_robot = [0, current_coordinates] # 0 - robot's coordinates, 1 - block coordinates 
    message_robot = struct.pack("4f", *message_robot)
    emitter.send(message_robot)

    #receive other robot's coordinates
    #print('Receiver Queue length:'  , receiver.getQueueLength())
    if receiver.getQueueLength() > 0:
        message = receiver.getData()
        message = list(struct.unpack("4f",message))
        if message[0] == 0:
            other_robot_coordinates = message[1:]
        elif message[0] == 1:
            list_of_blocks.append(message[1:])
        
        print('Red robot location:', message)
        receiver.nextPacket() #deletes the head packet
    # Process sensor data here.
    else:
        print('no message')

    distance_btw_robots = np.linalg.norm(np.array(current_coordinates) - np.array(other_robot_coordinates))
    print(distance_btw_robots)
    # detect obstacles
    right_obstacle = ds_1_value < 1000.0
    left_obstacle = ds_2_value < 1000.0

    # Check if the robot is near home
    if np.linalg.norm(np.array(current_coordinates) - np.array(home)) < 0.1:
        atHome = True
    else:
        atHome = False
    
    """ Unloading """
    if atHome and robot.getTime() > 8:  # If the robot is near home after 'return_to_home()'
        unloading = True                # Initiate unloading procedure
        reverse_coords = calc_reverse_coords(current_coordinates, current_bearing)

    #call obstacle_check only if obstacle outside of while loop and unloading is false
    #potentially changes obstacle to True if it detects a block
    if obstacle == False and unloading == False:
        if right_obstacle == True:
            block_coords, obstacle = obstacle_check('ds_1', obstacle)
        else:
            pass
        if left_obstacle == True:
            block_coords, obstacle = obstacle_check('ds_2', obstacle)
        else:
            pass
    else:
        pass

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
    MAX_SPEED = 6.28
    
    if unloading == False:
        #obstacle Boolean here might be different from the obstacle boolean at the start of this loop due to the previous if statement
        if obstacle == True and goinghome == False:
            alignment = False
            leftSpeed, rightSpeed, alignment = rotateTo(previous_coordinates, current_coordinates, block_coords, current_bearing, alignment)
            if alignment == True:
                colour = getRGB()
                alignment = False #switch alignment back to false
                print(colour)
                if colour == robot_colour: #implement collection function
                    print('yeboi collect it')
                    leftSpeed, rightSpeed, j = moveTo(previous_coordinates, current_coordinates, block_coords, current_bearing, i)
                    if j == i+1: #collected block
                        obstacle = False
                        goinghome = True
                        path.insert(i+2,home)
                elif colour == other_robot_colour: #implement avoidance function
                    print('nah screw you')
                elif colour == None:
                    print('cant determine')
                #elif colour == 1: 
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

# Enter here exit cleanup code.
