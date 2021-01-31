import math
import numpy as np

def getBearing(north):
    """
    Returns the bearing based off the compass north reading
    """
    rad = math.atan2(north[0],north[2])
    bearing = (rad - 1.5708)/math.pi * 180
    if bearing < 0.0:
        bearing = bearing + 360.0
    return bearing

def get_gps_xz():
    """
    Returns the gps coordinates with the y-coordinate removed
    """
    coords = gps.getValues()
    xz_coords = coords.pop(1)
    return xz_coords

def getDistanceandRotation(subpath):
    """
    Returns the distance between the next point and current point and rotation needed to align with the new direction.

    Arguments: Accepts a nested list of length  3
    [[prev point],[current point],[next point]]
    """
    subpath = np.array(subpath)

    if subpath.shape[0] == 3:
        prev_vector = subpath[1] - subpath[0]
        new_vector = subpath[2] - subpath[1]
        distance = np.linalg.norm(new_vector)

        unit_prev_vector = prev_vector / np.linalg.norm(prev_vector)
        unit_new_vector = new_vector / np.linalg.norm(new_vector)
        angle_prev_vec = np.arctan(unit_prev_vector[2]/unit_prev_vector[0])/math.pi * 180
        if unit_new_vector[0] == 0: # to avoid divide by zero problems
            if unit_new_vector[2] > 0:
                angle_new_vec = -90
            if unit_new_vector[2] < 0:
                angle_new_vec = 90
        else:
            angle_new_vec = np.arctan(unit_new_vector[2]/unit_new_vector[0])/math.pi * 180

        dot_product = np.dot(unit_prev_vector, unit_new_vector)
        if dot_product < 0:
            if angle_new_vec < 0:
                angle_new_vec += 180
            elif angle_new_vec > 0:
                angle_new_vec -= 180
        #angle = np.arccos(dot_product)
        #print(angle_prev_vec)
        #print(angle_new_vec)
        angle = angle_new_vec - angle_prev_vec

    else:
        print('wrong size')

    return distance, angle

def moveTo(previous_coordinates, current_coordinates, desired_coordinates, current_bearing, i):
    """
    Returns the leftSpeed and rightSpeed to move it in the correct direction and new i if it reaches the desired coordinate

    Arguments: previous_coordinates, current_coordinates, desired_coordinates, current_bearing, i
    Returns: leftSpeed, rightSpeed, i
    """
    MAX_SPEED = 6.28
    coordinates_list = [previous_coordinates,current_coordinates,desired_coordinates]
    distance, x = getDistanceandRotation(coordinates_list)

    # calculating desired bearing to get to desired coordinate from current coordinate
    ref_coordinates = [current_coordinates[0]+1, current_coordinates[1] , current_coordinates[2]] # to make previous vector always be [-1,0,0]
    coordinates_list2 = [ref_coordinates,current_coordinates,desired_coordinates]
    x,angle = getDistanceandRotation(coordinates_list2)
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

    if bearing_error < 1 and bearing_error > -1:
        if distance < 0.1: #stop once desired coordinate is nearby
            leftSpeed  = 0
            rightSpeed = 0
            i += 1
        else: #if no bearing error and not near desired coordinate, just go straight
            leftSpeed  = 0.5 * MAX_SPEED
            rightSpeed = 0.5 * MAX_SPEED
    elif bearing_error >= 1: #rotate right to reduce bearing error
            leftSpeed  = 0.5 * MAX_SPEED
            rightSpeed = -0.5 * MAX_SPEED
    elif bearing_error <= -1: #rotate left to reduce bearing error
            leftSpeed  = -0.5 * MAX_SPEED
            rightSpeed = 0.5 * MAX_SPEED

    return leftSpeed, rightSpeed, i
