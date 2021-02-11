import numpy as np
import math

# import sensors_api# import getCoordinates, getBearing, getRGB, get_attributes, ds_read
from .sensors_api import *
from .motion_api import bearing_round
from .variables import deg2rad, block_width, wall_list_x, other_colour_blocks

def find_obstacle_coords(ds, gps, compass):
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
    coordinates = getCoordinates(gps)
    bearing = getBearing(compass)
    #find absolute angle of detector. remember to convert to radians!
    ds_absolute_angle = (bearing + ds_attributes[1]) * (deg2rad)
    ds_absolute_disp_angle = (bearing + ds_attributes[2]) * (deg2rad)
    #print(ds_absolute_angle, ds_absolute_disp_angle)
    #find coordinates.
    x_coord = (ds_reading * np.cos(ds_absolute_angle)) + (ds_distance * np.cos(ds_absolute_disp_angle)) + coordinates[0]
    z_coord = (ds_reading * np.sin(ds_absolute_angle)) + (ds_distance * np.sin(ds_absolute_disp_angle)) + coordinates[1]
    
    obstacle_coord = np.array([x_coord, z_coord])

    return obstacle_coord

def find_block_coords(prelim_coords, bearing, ds_object):
    """
    A function which returns the coordinates of the centre of a block.

    Arguments: prelim_coords (the incident point's coordinates), bearing(the
    robot's bearing in degrees), ds (the distance sensor which detected an
    obstacle)
    """
    #find out the cartesian direction of the robot.
    cartesian_bearing = bearing_round(bearing)
    # print(type(cartesian_bearing))
    #use diagonal distance from corner to centre of block
    block_diagonal = (block_width/math.sqrt(2))/100
    #the corner detected will depend on the cartesian direction of the robot and
    #the distance sensor which detected the block (as they point in opposing
    #directions)
    if ds_object.getName() == 'ds_left':
        #find angle of corner to centre. remember to convert to radians!
        diagonal_absolute_angle = (bearing_round(bearing) - 45) * (deg2rad)
        x_block = prelim_coords[0] + block_diagonal * np.cos(diagonal_absolute_angle)
        z_block = prelim_coords[1] + block_diagonal * np.sin(diagonal_absolute_angle)
    elif ds_object.getName() == 'ds_right':
        diagonal_absolute_angle = (bearing_round(bearing) + 45) * (deg2rad)
        x_block = prelim_coords[0] + block_diagonal * np.cos(diagonal_absolute_angle)
        z_block = prelim_coords[1] + block_diagonal * np.sin(diagonal_absolute_angle)
    else:
        print('distance sensor not found')

    block_coords = np.array([x_block, z_block])
    print('Found block at:', block_coords)
    return block_coords

def obstacle_check(ds, gps, compass, obstacle, other_colour_blocks):
    """
    A function called if the distance sensors detect an object within the sweep lane. It determines whether the object
    is a block or a wall, calling the reciprocating_sweep function if it is a block.

    Arguments: ds (a string which denotes the distance sensor which detected an obstacle)
    Returns: block_coords, obstacle (Boolean)
    """

    prelim_coords = find_obstacle_coords(ds,gps,compass)
    
    wall_coord = wall_list_x[1]
    obstacle_tolerance = 0.01
    lower_wall = wall_coord - obstacle_tolerance
    upper_wall = wall_coord + obstacle_tolerance
    
    if lower_wall <= abs(prelim_coords[0]) <= upper_wall or lower_wall <= abs(prelim_coords[1]) <= upper_wall:
        # print('All okay! Just a wall')
        obstacle = False
        block_coords = None
        pass
    
    else:
        #check if the object has already been recorded
        obstacle = True
        for coords in other_colour_blocks:
            if ((coords[0] - 2.5 - obstacle_tolerance) <= coords[0] <= (coords[0] + 2.5 + obstacle_tolerance) and
                    (coords[1] - 2.5 - obstacle_tolerance) <= (coords[1] + 2.5 + obstacle_tolerance)):
                print('Deja vu!')
                obstacle = False

        # print('thats no moon!')
        if obstacle == True:
            block_coords = find_block_coords(prelim_coords, getBearing(compass), ds)

    return block_coords, obstacle

def avoid_obstacle(current_coordinates, block_coords, current_bearing):
    """
    Provides code to move past blocks that are not of correct colour
    """
    robot_size = [0.3, 0.2] #placeholder for actual size
    obs_size = 0.05 #placeholder for actual size
    dist_to_obj = np.sqrt((current_coordinates[0]-block_coords[0])**2 + (current_coordinates[1]-block_coords[1])**2)
    
    direction = bearing_round(current_bearing) #makes it 0,90,180,270
    if direction == 360:
        direction = 0
    
    if direction == 0:
        if current_coordinates[1] >= 0:
            #new points in -z direction
            new1 = [current_coordinates[0], current_coordinates[1]-(obs_size/2+robot_size[0])]
            new2 = [ new1[0]+(obs_size+dist_to_obj+robot_size[1]),new1[1]]
            new3 = [new2[0],current_coordinates[1]]
        else:
            #new points in z direction
            new1 = [current_coordinates[0], current_coordinates[1]+(obs_size/2+robot_size[0])]
            new2 = [ new1[0]+(obs_size+dist_to_obj+robot_size[1]),new1[1]]
            new3 = [new2[0],current_coordinates[1]]

    elif direction == 180:
        if current_coordinates[1] >= 0:
            #new points in -z direction
            new1 = [current_coordinates[0], current_coordinates[1]-(obs_size/2+robot_size[0])]
            new2 = [ new1[0]-(obs_size+dist_to_obj+robot_size[1]),new1[1]]
            new3 = [new2[0],current_coordinates[1]]
        else:
            #new points in z direction
            new1 = [current_coordinates[0], current_coordinates[1]+(obs_size/2+robot_size[0])]
            new2 = [ new1[0]-(obs_size+dist_to_obj+robot_size[1]),new1[1]]
            new3 = [new2[0],current_coordinates[1]]

    elif direction == 90 :
        if current_coordinates[0] >= 0:
            #new points in the -x direction
            new1 = [current_coordinates[0]-(obs_size/2+robot_size[0]), current_coordinates[1]]
            new2 = [new1[0],new1[1]+(obs_size+dist_to_obj+robot_size[1])]
            new3 = [current_coordinates[0], new2[1]]
        else:
            #new points in the x direction
            new1 = [current_coordinates[0]+(obs_size/2+robot_size[0]), current_coordinates[1]]
            new2 = [new1[0],new1[1]+(obs_size+dist_to_obj+robot_size[1])]
            new3 = [current_coordinates[0], new2[1]]

    elif direction == 270 :
        if current_coordinates[0] >= 0:
            #new points in the -x direction
            new1 = [current_coordinates[0]-(obs_size/2+robot_size[0]), current_coordinates[1]]
            new2 = [new1[0],new1[1]-(obs_size+dist_to_obj+robot_size[1])]
            new3 = [current_coordinates[0], new2[1]]
        else:
            #new points in the x direction
            new1 = [current_coordinates[0]+(obs_size/2+robot_size[0]), current_coordinates[1]]
            new2 = [new1[0],new1[1]-(obs_size+dist_to_obj+robot_size[1])]
            new3 = [current_coordinates[0], new2[1]]

    bypass_points = np.array([new1,new2,new3])

    return bypass_points