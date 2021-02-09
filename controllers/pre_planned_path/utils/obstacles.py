import numpy as np 

def avoid_obstacle(curr_coord, obs_coord):
    """
    Provides code to move past blocks that are not of correct colour

    :param curr_coord: current coordinates of the vehicle
    :type curr_coord: int array
    :param obs_coord: coordinates of the obstacle
    :type obs_coord: int array
    :param obs_size: size of the obstacle
    :type obs_size: float
    """

    robot_size = [10, 10] #placeholder for actual size
    obs_size = 10 #placeholder for actual size

    dist_to_obj = np.sqrt((curr_coord[0]-obs_coord[0])^2 + (curr_coord[1]-obs_coord[1])^2)

    new1 = [curr_coord[0]+obs_size/2+robot_size[0], curr_coord[1]]
    new2 = [new1[0], new1[1]+obs_size+2*dist_to_obj+robot_size[1]]
    new3 = [curr_coord[0], new2[1]]

    return [new1, new2, new3]
