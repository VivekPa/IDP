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
        angle_prev_vec = np.arctan(unit_prev_vector[2]/unit_prev_vector[0])
        angle_new_vec = np.arctan(unit_new_vector[2]/unit_new_vector[0])
        print(angle_prev_vec)
        print(angle_new_vec)
        
        #ot_product = np.dot(unit_prev_vector, unit_new_vector)
        #angle = np.arccos(dot_product)
        angle = angle_new_vec - angle_prev_vec
        if math.isnan(angle):
            angle = 0.0
    
    else:
        print('wrong size')

    return distance, angle/math.pi * 180