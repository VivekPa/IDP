import math
import numpy as np

from .variables import MAX_SPEED, deg2rad
from .variables import deg2rad, block_width, wall_list_x, other_colour_blocks, home

def bearing_round(bearing, base = 90):
    """
    A functioon which returns the bearing rounded to the nearest 90 degrees.

    Returns: rounded_bearing
    """
    rounded_bearing = base * round(bearing / base)
    return rounded_bearing

def getDistanceandRotation(subpath):
    """
    A function which returns the distance between the next point and current point and rotation needed to align with the new direction.

    Arguments: Accepts a ndarray of length  3
    [[prev point],[current point],[next point]]
    Returns: distance, angle
    """

    prev_vector = subpath[1] - subpath[0]
    new_vector = subpath[2] - subpath[1]
    distance = np.linalg.norm(new_vector)
    angle_prev_vec = 0
    unit_prev_vector = prev_vector / np.linalg.norm(prev_vector)
    if np.linalg.norm(new_vector) == 0:
        unit_new_vector = new_vector / (np.linalg.norm(new_vector)+0.01)
    else:
        unit_new_vector = new_vector / np.linalg.norm(new_vector)
    angle_prev_vec = np.arctan(unit_prev_vector[1]/unit_prev_vector[0])/math.pi * 180
    
    if unit_new_vector[0] == 0: # to avoid divide by zero problems
        if unit_new_vector[1] > 0:
            angle_new_vec = -90
        if unit_new_vector[1] < 0:
            angle_new_vec = 90
        if unit_new_vector[1] == 0:
            angle_new_vec = 0
    else:
        angle_new_vec = np.arctan(unit_new_vector[1]/unit_new_vector[0])/math.pi * 180

    dot_product = np.dot(unit_prev_vector, unit_new_vector)
    if dot_product < 0:
        if angle_new_vec < 0:
            angle_new_vec += 180
        elif angle_new_vec > 0:
            angle_new_vec -= 180
    
    angle = angle_new_vec - angle_prev_vec

    return distance, angle

def moveTo(previous_coordinates, current_coordinates, desired_coordinates, current_bearing, a):
    """
    A function which returns the leftSpeed and rightSpeed to move it in the correct direction and new a if it reaches the desired coordinate

    Arguments: previous_coordinates, current_coordinates, desired_coordinates, current_bearing, a
    Returns: leftSpeed, rightSpeed, a
    """
    # MAX_SPEED = 6.28
    leftSpeed  = 0
    rightSpeed = 0
    coordinates_list = [previous_coordinates,current_coordinates,desired_coordinates]
    distance = getDistanceandRotation(coordinates_list)[0]

    # calculating desired bearing to get to desired coordinate from current coordinate
    ref_coordinates = [current_coordinates[0]+1, current_coordinates[1]] # to make previous vector always be [-1,0]
    coordinates_list2 = [ref_coordinates,current_coordinates,desired_coordinates]
    angle = getDistanceandRotation(coordinates_list2)[1]
    desired_bearing = angle + 180

    bearing_error = desired_bearing - current_bearing

    # bearing +- 180 to get the smallest bearing error in the correct direction
    if bearing_error > 180:
        bearing_error -= 360
    elif bearing_error < -180:
        bearing_error += 360

    if bearing_error < 0.5 and bearing_error > -0.5:
        if distance < 0.06: #stop once desired coordinate is nearby
            leftSpeed  = 0
            rightSpeed = 0
            a += 1
        elif distance < 0.8: #stop once desired coordinate is nearby
            leftSpeed  = 0.3 * MAX_SPEED
            rightSpeed = 0.3 * MAX_SPEED
        else: #if no bearing error and not near desired coordinate, just go straight
            leftSpeed  = 1.0 * MAX_SPEED
            rightSpeed = 1.0 * MAX_SPEED
    elif bearing_error >= 0.5: #rotate right to reduce bearing error
            leftSpeed  = 0.5 * MAX_SPEED
            rightSpeed = -0.5 * MAX_SPEED
    elif bearing_error <= -0.5: #rotate left to reduce bearing error
            leftSpeed  = -0.5 * MAX_SPEED
            rightSpeed = 0.5 * MAX_SPEED

    return leftSpeed, rightSpeed, a

def rotateTo(previous_coordinates, current_coordinates, desired_coordinates, current_bearing, alignment):
    """
    A function which returns the leftSpeed and rightSpeed to rotate it to face the next coordinates and new A if it is aligned

    Arguments: previous_coordinates, current_coordinates, desired_coordinates, current_bearing, a
    Returns: leftSpeed, rightSpeed, a
    """
    # calculating desired bearing to get to desired coordinate from current coordinate
    ref_coordinates = np.array([current_coordinates[0]+1, current_coordinates[1]]) # to make previous vector always be [-1,0,0]
    coordinates_list2 = [ref_coordinates,current_coordinates,desired_coordinates]
    angle = getDistanceandRotation(coordinates_list2)[1]
    desired_bearing = angle + 180

    bearing_error = desired_bearing - current_bearing

    # bearing +- 180 to get the smallest bearing error in the correct direction
    if bearing_error > 180:
        bearing_error -= 360
    elif bearing_error < -180:
        bearing_error += 360

    if bearing_error < 0.5 and bearing_error > -0.5:
        leftSpeed  = 0
        rightSpeed = 0
        alignment = True
    elif bearing_error >= 0.5: #rotate right to reduce bearing error
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = -0.5 * MAX_SPEED
    elif bearing_error <= -0.5: #rotate left to reduce bearing error
        leftSpeed  = -0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED

    return leftSpeed, rightSpeed, alignment

def reverseTo(previous_coordinates, current_coordinates, reverse_coords, a):
    """
    A function that returns the leftSpeed and rightSpeed to reverse until it reaches the required coords and new a once it reaches the point

    Arguments: previous_coordinates, current_coordinates, desired_coordinates, a
    Returns: leftSpeed, rightSpeed, a
    """
    coordinates_list = np.array([previous_coordinates, current_coordinates, reverse_coords])
    distance = getDistanceandRotation(coordinates_list)[0]

    if distance < 0.08: #stop once desired coordinate is nearby
        leftSpeed  = 0
        rightSpeed = 0
        a += 1

    else: #just reverse
            leftSpeed  = -0.7 * MAX_SPEED
            rightSpeed = -0.7 * MAX_SPEED

    return leftSpeed, rightSpeed, a

def calc_reverse_coords(current_coordinates, current_bearing):
    """
    A function that calculates the coordinates it needs to reverse to when wanting to unload
    Arguments: current_coordinates, current_bearing
    Returns: reverse_coords
    """
    if current_bearing >= 180:
        desired_bearing = current_bearing - 180
    elif current_bearing < 180:
        desired_bearing = current_bearing + 180
    desired_bearing = desired_bearing * (deg2rad)
    reverse_coords = np.array([ current_coordinates[0] + 0.4*np.cos(desired_bearing),
                                current_coordinates[1] + 0.4*np.sin(desired_bearing)])

    wall_coord = wall_list_x[1]
    obstacle_tolerance = 0.06
    lower_wall = wall_coord - obstacle_tolerance
    upper_wall = wall_coord + obstacle_tolerance
    
    if lower_wall <= abs(reverse_coords[0]) <= upper_wall or lower_wall <= abs(reverse_coords[1]) <= upper_wall:
        reverse_coords = np.array([0.551,1.12])
    return reverse_coords

def calc_collection_coords(block_coords, current_bearing):
    """
    A function that calculates the coordinates it needs to move to collect the block (did not use it in the end)
    Arguments: block_coords, current_bearing
    Returns: collection_coords
    """
    offset_distance = 0.00
    desired_bearing = current_bearing * (deg2rad)
    collection_coords = np.array([block_coords[0] + offset_distance*np.cos(desired_bearing),
                                block_coords[1] + offset_distance*np.sin(desired_bearing)])

    return collection_coords


