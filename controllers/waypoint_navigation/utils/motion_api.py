import math
import numpy as np

from .variables import MAX_SPEED, deg2rad

def bearing_round(bearing, base = 90):
    """
    Returns the bearing rounded to the nearest 90 degrees.
    """
    rounded_bearing = base * round(bearing / base)
    return rounded_bearing

def getDistanceandRotation(subpath):
    """
    Returns the distance between the next point and current point and rotation needed to align with the new direction.

    Arguments: Accepts a ndarray of length  3
    [[prev point],[current point],[next point]]
    """
    # print(np.round(subpath[0]))
    # print(np.round(subpath[1]))
    # print(np.round(subpath[2]))

    prev_vector = subpath[1] - subpath[0]
    new_vector = subpath[2] - subpath[1]
    distance = np.linalg.norm(new_vector)

    unit_prev_vector = prev_vector / np.linalg.norm(prev_vector)
    unit_new_vector = new_vector / np.linalg.norm(new_vector)
    angle_prev_vec = np.arctan(unit_prev_vector[1]/unit_prev_vector[0])/math.pi * 180

    if unit_new_vector[0] == 0: # to avoid divide by zero problems
        if unit_new_vector[1] > 0:
            angle_new_vec = -90
        if unit_new_vector[1] < 0:
            angle_new_vec = 90
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

def moveTo(previous_coordinates, current_coordinates, desired_coordinates, current_bearing, i):
    """
    Returns the leftSpeed and rightSpeed to move it in the correct direction and new i if it reaches the desired coordinate

    Arguments: previous_coordinates, current_coordinates, desired_coordinates, current_bearing, i
    Returns: leftSpeed, rightSpeed, i
    """
    # MAX_SPEED = 6.28
    coordinates_list = [previous_coordinates,current_coordinates,desired_coordinates]
    distance = getDistanceandRotation(coordinates_list)[0]

    # calculating desired bearing to get to desired coordinate from current coordinate
    ref_coordinates = [current_coordinates[0]+1, current_coordinates[1]] # to make previous vector always be [-1,0]
    coordinates_list2 = [ref_coordinates,current_coordinates,desired_coordinates]
    angle = getDistanceandRotation(coordinates_list2)[1]
    # print('angle',angle)
    desired_bearing = angle + 180

    # print(i,distance, desired_bearing)
    # print('desired bearing', desired_bearing)
    bearing_error = desired_bearing - current_bearing
    # print('bearing error', bearing_error)

    # bearing +- 180 to get the smallest bearing error in the correct direction
    if bearing_error > 180:
        bearing_error -= 360
    elif bearing_error < -180:
        bearing_error += 360

    if bearing_error < 0.5 and bearing_error > -0.5:
        if distance < 0.06: #stop once desired coordinate is nearby
            leftSpeed  = 0
            rightSpeed = 0
            i += 1
        elif distance < 1.0: #stop once desired coordinate is nearby
            leftSpeed  = 0.3 * MAX_SPEED
            rightSpeed = 0.3 * MAX_SPEED
        else: #if no bearing error and not near desired coordinate, just go straight
            leftSpeed  = 0.5 * MAX_SPEED
            rightSpeed = 0.5 * MAX_SPEED
    elif bearing_error >= 0.5: #rotate right to reduce bearing error
            leftSpeed  = 0.5 * MAX_SPEED
            rightSpeed = -0.5 * MAX_SPEED
    elif bearing_error <= -0.5: #rotate left to reduce bearing error
            leftSpeed  = -0.5 * MAX_SPEED
            rightSpeed = 0.5 * MAX_SPEED

    return leftSpeed, rightSpeed, i

def rotateTo(previous_coordinates, current_coordinates, desired_coordinates, current_bearing, alignment):
    """
    Returns the leftSpeed and rightSpeed to rotate it to face the next coordinates

    Arguments: previous_coordinates, current_coordinates, desired_coordinates, current_bearing, i
    Returns: leftSpeed, rightSpeed, i
    """
    # calculating desired bearing to get to desired coordinate from current coordinate
    ref_coordinates = np.array([current_coordinates[0]+1, current_coordinates[1]]) # to make previous vector always be [-1,0,0]
    coordinates_list2 = [ref_coordinates,current_coordinates,desired_coordinates]
    angle = getDistanceandRotation(coordinates_list2)[1]
    # print('angle',angle)
    desired_bearing = angle + 180

    # print(i,distance, desired_bearing)
    #print('desired bearing', desired_bearing)
    bearing_error = desired_bearing - current_bearing
    #print('bearing error', bearing_error)

    # bearing +- 180 to get the smallest bearing error in the correct direction
    if bearing_error > 180:
        bearing_error -= 360
    elif bearing_error < -180:
        bearing_error += 360

    if bearing_error < 0.5 and bearing_error > -0.5:
        leftSpeed  = 0
        rightSpeed = 0
        alignment = True
        # print('aligned')
    elif bearing_error >= 0.5: #rotate right to reduce bearing error
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = -0.5 * MAX_SPEED
    elif bearing_error <= -0.5: #rotate left to reduce bearing error
        leftSpeed  = -0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED

    return leftSpeed, rightSpeed, alignment

def reverseTo(previous_coordinates, current_coordinates, reverse_coords, i):
    """
    Returns the leftSpeed and rightSpeed to reverse until it reaches the required coords

    Arguments: previous_coordinates, current_coordinates, desired_coordinates, i
    Returns: leftSpeed, rightSpeed, i
    """
    coordinates_list = np.array([previous_coordinates, current_coordinates, reverse_coords])
    distance = getDistanceandRotation(coordinates_list)[0]

    if distance < 0.08: #stop once desired coordinate is nearby
        leftSpeed  = 0
        rightSpeed = 0
        i += 1

    else: #just reverse
            leftSpeed  = -0.5 * MAX_SPEED
            rightSpeed = -0.5 * MAX_SPEED

    return leftSpeed, rightSpeed, i

def calc_reverse_coords(current_coordinates, current_bearing):
    """
    Calculates the coordinates it needs to reverse to when wanting unload
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

    return reverse_coords

def stop(leftMotor, rightMotor):

    """
    This function stops the robot.
    """
    leftMotor.setVelocity(0.0)
    rightMotor.setVelocity(0.0)

def return_to_home(path, i, home):
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

def find_bearing(current_coords, desired_coords):
    """
    This function returns the bearing of the line between two given coordinates.
    """
    #find vector of travel
    relative_vector = desired_coords - current_coords
    acw_from_z = arctan2(relative_vector[2], relative_vector[0]) / deg2rad
    bearing = 90 - acw_from_z
    if bearing <= 360:
        bearing += 360
    return bearing

def find_path(current_coords, desired_coords):
    block_radius = (block_width / np.sqrt(2)) * 0.01
    #define path step
    path_step = 0.001
    #find vector of travel
    relative_vector = desired_coords - current_coords
    #find total distance
    d_total = np.linalg.norm(relative_vector)
    #normalise to give a direction vector
    direction_vector = relative_vector/d_total
    #find length of iterations to make for the given step
    d_iteration = path_step * np.floor(d_total / path_step)
    #now iterate through lambda until collides with a block
    for lambda in range(0, path_step, d_iteration):
        #find the coordinates on the line at this lambda
        path_coords = current_coords + lambda * direction_vector
        #now check if it collides with any block
        for block_coords in list_of_blocks:
            current_block_vector = block_coords - path_coords
            d_relative_block = np.linalg.norm(relative_block_vector)
            #if it collides with a block, find the tangents to route around it
            if d_relative_block < 0.12:
                block_vector1 = block_coords - current_coords
                d1_block = np.linalg.norm(block_vector1)
                tangent_angle1 = np.arcsin(block_radius / d1_block)
                tangent_point1 =

    return 1
