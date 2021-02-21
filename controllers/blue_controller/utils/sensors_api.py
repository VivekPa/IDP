import math
import numpy as np

ds_value = 0

def get_attributes(ds_object):
    """
    A function which returns the distance sensor location and orientation values
    given the name of a distance sensor. Workaround for functions not taking
    objects as arguments.

    Arguments: ds (a string which denotes the desired distance sensor)

    Note: Angles given are taken from the z-axis of the robot.
    """
    if ds_object.getName() == 'ds_left':
        ds_distance = 0.1089770618
        ds_angle = 60
        ds_disp_angle = -42.76882539
    elif ds_object.getName() == 'ds_right':
        ds_distance = 0.1089770618
        ds_angle = -60
        ds_disp_angle = 42.76882539
    else:
        print("distance sensor not found")
    return ds_distance, ds_angle, ds_disp_angle

def ds_read(ds_object, ds_value=0):
    """
    A function which returns the correct distance sensor reading given the name
    of a distance sensor. Workaround for functions not taking objects as
    arguments.

    Arguments: ds (a string which denotes the desired distance sensor)
    """
    if ds_object.getName() == 'ds_left':
        ds_value = 0.0003 * ds_object.getValue() + 0.011
    elif ds_object.getName() == 'ds_right':
        ds_value = 0.0003 * ds_object.getValue() + 0.011
    else:
        print('distance sensor not found')
    # print(ds_value)
    return ds_value

def getRGB(camera_left, camera_right):
    """
    A function that returns the integer corresponding to colour detected by the camera sensors
        0: red
        1: green
        2: blue
        None: one red and one blue
    
    Returns: colour
    """
    image_left = camera_left.getImageArray()
    image_right = camera_right.getImageArray()
    RGB_left = image_left[0][0]
    RGB_right = image_right[0][0]
    colour_left = RGB_left.index(max(RGB_left))
    colour_right = RGB_right.index(max(RGB_right))
    if colour_left == colour_right:
        colour = colour_left
    elif colour_left != 1 and colour_right != 1:
        print('error! one blue one red')
        colour = None
    else:
        if colour_left == 1:
            colour = colour_right
        else:
            colour = colour_left

    return colour

def getBearing(compass):
    """
    A function that returns the bearing based off the compass north reading
    Returns: bearing
    """
    compassData = compass.getValues()
    rad = math.atan2(compassData[0], compassData[2])
    bearing = (rad - math.pi/2)/math.pi * 180
    if bearing < 0.0:
        bearing = bearing + 360.0
    return bearing

def getCoordinates(gps):
    """
    A function that reduce the dimensions of the coordinates from 3 to 2
    Returns: xz coordinates
    """
    xyz     = np.array(gps.getValues())
    xz      = np.delete(xyz, 1, axis=0)
    return xz

