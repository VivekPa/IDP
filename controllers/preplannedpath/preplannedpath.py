"""preplannedpath controller."""

# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, DistanceSensor, GPS, Emitter, Receiver, Camera
from subdir.functions import getBearing, getDistanceandRotation, moveTo, get_gps_xz, bearing_round, rotateTo
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

camera = robot.getDevice('camera')
camera.enable(TIME_STEP)

home = [1,0,1]
path = [home,home,[0,0,1],[-1,0,1],[-1,0,0.6],[0,0,0.6],[1,0,0.6],[1,0,0.2],[0,0,0.2],[-1,0,0.2]] # always duplicate first point
#path = [[1,1], [1,1],[0,1],[1,1],[-1,0],[0,0],[-1,-1],[0,0]] # always duplicate first point

i = 0
obstacle = False
previous_coordinates = path[0]
robot_colour = 2 # 0 - red, 1 - green, 2- blue
other_robot_colour = 0

#initialise 'active block coordinates'
block_coords = []
#initialise block list
other_colour_blocks = []

#declare turn variable to decide on path home
path_turns = 0
turnpoints = [np.array([-1, 0, 0.6]), np.array([1, 0, 0.2])]

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
    ds_absolute_angle = (bearing + ds_attributes[1]) * (3.14159265358929323846264/180)
    ds_absolute_disp_angle = (bearing + ds_attributes[2]) * (3.14159265358929323846264/180)
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
        diagonal_absolute_angle = (bearing_round(bearing) - 45) * (3.14159265358979323846264/180)
        x_block = prelim_coords[0] + block_diagonal * np.cos(diagonal_absolute_angle)
        z_block = prelim_coords[1] + block_diagonal * np.sin(diagonal_absolute_angle)
    elif ds == 'ds_2':
        diagonal_absolute_angle = (bearing_round(bearing) + 45) * (3.14159265358979323846264/180)
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
    obstacle_tolerance = 0.01
    lower_wall = wall_coord - obstacle_tolerance
    upper_wall = wall_coord + obstacle_tolerance
    if lower_wall <= abs(x_prelim) <= upper_wall or lower_wall <= abs(z_prelim) <= upper_wall:
        print('All okay! Just a wall')
        obstacle = False
        block_coords = None
        pass
    else:
        #check if the object has already been recorded
        for coords in other_colour_blocks:
            if (coords[0] - 2.5 - obstacle_tolerance) <= x_coords <= (coords[0] + 2.5 + obstacle_tolerance) and (coords[2] - 2.5 - obstacle_tolerance) <= (coords[2] + 2.5 + obstacle_tolerance):
                print('Deja vu!')
                obstacle = False
                break
        else:
            #otherwise its a new obstacle
            print('Thats no moon!')
            obstacle = True
            block_coords = find_block_coords(prelim_coords, getBearing(compass.getValues()), ds)
    last_known_point = gps.getValues()

    return block_coords, obstacle, last_known_point

def stop():
    """
    This function stops the robot.
    """
    leftMotor.setVelocity(0.0)
    rightMotor.setVelocity(0.0)

"""
End of search functions
"""

def getRGB():
    """
    Returns: integer cooresponding to colour with largest pixel value as well individual pixel values
        0: red
        1: green
        2: blue
    """
    image = camera.getImageArray()
    RGB = image[0][0]
    colour = RGB.index(max(RGB))
    red   = RGB[0]
    green = RGB[1]
    blue  = RGB[2]

    return colour, red, green, blue


# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    if i == len(path)-2:
        print('reached the end')
        break

    # get current device values
    current_coordinates = np.array(gps.getValues())
    north = compass.getValues()
    current_bearing = getBearing(north)
    ds_1_value = ds_left.getValue()
    ds_2_value = ds_right.getValue()

    # detect obstacles
    right_obstacle = ds_1_value < 1000.0
    left_obstacle = ds_2_value < 1000.0

    #call obstacle_check only if obstacle outside of while loop is false
    #potentially changes obstacle to True if it detects a block
    if obstacle == False:
        if right_obstacle == True:
            block_coords, obstacle, last_known_point = obstacle_check('ds_1', obstacle)
        else:
            pass
        if left_obstacle == True:
            block_coords, obstacle, last_known_point = obstacle_check('ds_2', obstacle)
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

    #check if robot has made a turn
    coordinates = path[i+2]
    for turnpoint in turnpoints:
        for i in range(0,2,1):
            if coordinates[i] == turnpoint[i]:
                turn = True
            else:
                turn = False
                break
        if turn == True:
            path_turns += 1
            turnpoints.pop(0)
            print("turned")

    MAX_SPEED = 6.28
    #obstacle Boolean here might be different from the obstacle boolean at the start of this loop due to the previous if statement
    if obstacle == True:
        alignment = False
        leftSpeed, rightSpeed, alignment = rotateTo(previous_coordinates, current_coordinates, block_coords, current_bearing, alignment)
        if alignment == True:
            colour, red, green, blue = getRGB()
            alignment = False #switch alignment back to false
            print(colour, red, green, blue)
            if colour == robot_colour: #implement collection function
                print('yeboi collect it')
                leftSpeed, rightSpeed, j = moveTo(previous_coordinates, current_coordinates, block_coords, current_bearing, i)
                if j == i+1: #collected block
                    obstacle = False
                    path.insert(i+2, last_known_point)
                    if path_turns == 0:
                        pass #path.insert(i+3, home)
                    else:
                        path.insert(i+3, [last_known_point[0], 0, 1])
                        path.insert(i+4, np.array(home))
            elif colour == other_robot_colour: #implement avoidance function
                print('nah screw you')
                #append to the list of other colour blocks
                other_colour_blocks.append(block_coordinates)
            # leftSpeed  = 0
            # rightSpeed = 0

            #obstacle = False #change obstacle back to False after collecting the block
            #print(obstacle)
            # leftMotor.setVelocity(leftSpeed)
            # rightMotor.setVelocity(rightSpeed)
            # print('trying to break')
            # break
    else:
        leftSpeed, rightSpeed, i = moveTo(previous_coordinates, current_coordinates, desired_coordinates, current_bearing, i)
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    previous_coordinates = current_coordinates

# Enter here exit cleanup code.
