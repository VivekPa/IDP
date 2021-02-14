import numpy as np
import math

# import sensors_api# import getCoordinates, getBearing, getRGB, get_attributes, ds_read
from .sensors_api import *
from .motion_api import bearing_round
from .variables import deg2rad, block_width, wall_list_x, other_colour_blocks, home

def bearing_floor(bearing, floor=90):
    """
    A function which returns the floor of a bearing with respect to 90 degrees.
    """
    floored_bearing = 90 * np.floor(bearing / 90)
    return floored_bearing


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

def obstacle_check(ds, gps, compass, obstacle, other_colour_blocks, indetermined_obs_blocks):
    """
    A function called if the distance sensors detect an object within the sweep lane. It determines whether the object
    is a block or a wall, calling the reciprocating_sweep function if it is a block.

    Arguments: ds (a string which denotes the distance sensor which detected an obstacle)
    Returns: block_coords, obstacle (Boolean)
    """

    prelim_coords = find_obstacle_coords(ds,gps,compass)
    
    wall_coord = wall_list_x[1]
    obstacle_tolerance = 0.06
    lower_wall = wall_coord - obstacle_tolerance
    upper_wall = wall_coord + obstacle_tolerance
    left_edge_home = home[1] - 0.2
    right_edge_home = home[1] + 0.2
    bottom_edge_home = home[0] - 0.2
    
    if lower_wall <= abs(prelim_coords[0]) <= upper_wall or lower_wall <= abs(prelim_coords[1]) <= upper_wall:
        print('All okay! Just a wall')
        obstacle = False
        block_coords = None
        pass

    elif bottom_edge_home <= prelim_coords[0] and left_edge_home <= prelim_coords[1] <= right_edge_home:
        print('All okay! blocks at home')
        obstacle = False
        block_coords = None
        pass
    else:
        #check if the object has already been recorded
        obstacle = True
        for coords in other_colour_blocks:
            if ((coords[0] - .025 - obstacle_tolerance) <= prelim_coords[0] <= (coords[0] + 0.025 + obstacle_tolerance) and
                    (coords[1] - 0.025 - obstacle_tolerance) <= prelim_coords[1] <= (coords[1] + 0.025 + obstacle_tolerance)):
                print('Deja vu!')
                obstacle = False

        for coords in indetermined_obs_blocks:
            if ((coords[0] - .025 - obstacle_tolerance) <= prelim_coords[0] <= (coords[0] + 0.025 + obstacle_tolerance) and
                    (coords[1] - 0.025 - obstacle_tolerance) <= prelim_coords[1] <= (coords[1] + 0.025 + obstacle_tolerance)):
                print('Deja vu!')
                obstacle = False

    
        block_coords = find_block_coords(prelim_coords, getBearing(compass), ds)

    return block_coords, obstacle

"""
new code for getting obstacles at an angle
"""
# def find_obstacle_coords(ds, gps, compass):
#     """
#     A function which returns the coordinates of the incident point on the
#     surface which the distance sensor is detecting.
#     Arguments: ds (a string which denotes the distance sensor which detected an
#     obstacle)
#     """
#     #make a rough estimate of coordinates of the point on a surface it 'sees
#     #retrieve reading
#     ds_reading = ds_read(ds)
#     #print(ds_reading)
#     #retrieve attributes
#     ds_attributes = get_attributes(ds)
#     ds_distance = ds_attributes[0]
#     # print(ds_distance)
#     #retrieve gps and bearing
#     coordinates = getCoordinates(gps)
#     bearing = getBearing(compass)
#     #find out the cartesian direction of the robot.
#     cartesian_bearing = bearing_round(bearing)
#     #find absolute angle of detector. remember to convert to radians!
#     ds_absolute_angle = (bearing + ds_attributes[1]) * (deg2rad)
#     ds_absolute_disp_angle = (bearing + ds_attributes[2]) * (deg2rad)
#     #print(ds_absolute_angle, ds_absolute_disp_angle)
#     #find coordinates.
#     x_coord = (ds_reading * np.cos(ds_absolute_angle)) + (ds_distance * np.cos(ds_absolute_disp_angle)) + coordinates[0]
#     z_coord = (ds_reading * np.sin(ds_absolute_angle)) + (ds_distance * np.sin(ds_absolute_disp_angle)) + coordinates[1]

#     obstacle_coords = np.array([x_coord, z_coord])

#     return obstacle_coords, ds_absolute_angle

# def find_block_coords(prelim_coords, ds_absolute_angle, ds_object):
#     """
#     A function which returns the coordinates of the centre of a block.
#     Arguments: prelim_coords (the incident point's coordinates), bearing(the
#     robot's bearing in degrees), ds (the distance sensor which detected an
#     obstacle)
#     """

#     #use diagonal distance from corner to centre of block
#     block_diagonal = (block_width/math.sqrt(2))/100
#     #the corner detected will depend on the cartesian direction of the robot and
#     #the distance sensor which detected the block (as they point in opposing
#     #directions)
#     floored_line_bearing = bearing_floor(ds_absolute_angle)
#     if ds_object.getName() == 'ds_left':
#         #find angle of corner to centre. remember to convert to radians!
#         diagonal_absolute_angle = (floored_line_bearing - 45) * (deg2rad)
#         x_block = prelim_coords[0] + block_diagonal * np.cos(diagonal_absolute_angle)
#         z_block = prelim_coords[1] + block_diagonal * np.sin(diagonal_absolute_angle)
#     elif ds_object.getName() == 'ds_right':
#         diagonal_absolute_angle = (floored_line_bearing + 135) * (deg2rad)
#         x_block = prelim_coords[0] + block_diagonal * np.cos(diagonal_absolute_angle)
#         z_block = prelim_coords[1] + block_diagonal * np.sin(diagonal_absolute_angle)
#     else:
#         print('distance sensor not found')

#     block_coords = np.array([x_block, z_block])
#     print('Found block at:', block_coords)
#     return block_coords

# def obstacle_check(ds, gps, compass, obstacle, other_colour_blocks, indetermined_obs_blocks):
#     """
#     A function called if the distance sensors detect an object within the sweep lane. It determines whether the object
#     is a block or a wall, calling the reciprocating_sweep function if it is a block.

#     Arguments: ds (a string which denotes the distance sensor which detected an obstacle)
#     Returns: block_coords, obstacle (Boolean)
#     """

#     # prelim_coords = find_obstacle_coords(ds,gps,compass)
#     prelim_coords, ds_absolute_angle = find_obstacle_coords(ds,gps,compass)
    
#     wall_coord = wall_list_x[1]
#     obstacle_tolerance = 0.03
#     lower_wall = wall_coord - obstacle_tolerance
#     upper_wall = wall_coord + obstacle_tolerance
#     left_edge_home = home[1] - 0.2
#     right_edge_home = home[1] + 0.2
#     bottom_edge_home = home[0] - 0.2
    
#     if lower_wall <= abs(prelim_coords[0]) <= upper_wall or lower_wall <= abs(prelim_coords[1]) <= upper_wall:
#         # print('All okay! Just a wall')
#         obstacle = False
#         block_coords = None
#         pass

#     elif bottom_edge_home <= abs(prelim_coords[0]) and left_edge_home <= abs(prelim_coords[1]) <= right_edge_home:
#         # print('All okay! blocks at home')
#         obstacle = False
#         block_coords = None
#         pass
#     else:
#         #check if the object has already been recorded
#         obstacle = True
#         for coords in other_colour_blocks:
#             if ((coords[0] - .025 - obstacle_tolerance) <= prelim_coords[0] <= (coords[0] + 0.025 + obstacle_tolerance) and
#                     (coords[1] - 0.025 - obstacle_tolerance) <= prelim_coords[1] <= (coords[1] + 0.025 + obstacle_tolerance)):
#                 print('Deja vu!')
#                 obstacle = False

#         for coords in indetermined_obs_blocks:
#             if ((coords[0] - .025 - obstacle_tolerance) <= prelim_coords[0] <= (coords[0] + 0.025 + obstacle_tolerance) and
#                     (coords[1] - 0.025 - obstacle_tolerance) <= prelim_coords[1] <= (coords[1] + 0.025 + obstacle_tolerance)):
#                 print('Deja vu!')
#                 obstacle = False

    
#         #block_coords = find_block_coords(prelim_coords, getBearing(compass), ds)
#         block_coords = find_block_coords(prelim_coords, ds_absolute_angle, ds)

#     return block_coords, obstacle