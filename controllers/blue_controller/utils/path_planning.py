from utils.astarplanner import AStarPlanner
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Rectangle
import numpy as np

move_straight = False
ARENA_WIDTH     = 240 #[cm]
HOME_WIDTH      = 40    #[cm]
deg2rad = 0.01745329252
block_width = 5
pass_distance = 0.12

def remove_redundant(xc, yc): #for straight movement
    """
    A function which removes redundant points for straight movement.
    Arguments: x_coords, y_coords (all points along the path)
    Returns: xc, yc (only the necessary points)
    """
        xc = xc.copy()
        yc = yc.copy()
        i = 0
        while i < len(xc) - 2:
            triplex = round(xc[i], 2) == round(xc[i+1], 2) and round(xc[i+1], 2) == round(xc[i+2], 2)
            tripley = round(yc[i] == yc[i+1], 2) and round(yc[i+1] == yc[i+2], 2)
            if triplex or tripley:
                xc.pop(i+1)
                yc.pop(i+1)
                i -= 1
            i += 1
        return xc, yc

def get_corners(rx, ry): #for movement with diagonals
    """
    A function which removes redundant points for movement with diagonals.
    Arguments: x_coords, y_coords (all points along the path)
    Returns: nrc, nry (only the necessary points)
    """
    nrx, nry = [], []
    for idx,value in enumerate(rx):
        if idx == len(rx)-1:
            nrx.append(value)
            nry.append(ry[idx])
            break
        else:
            diff_x = rx[idx+1]-rx[idx]
            diff_y = ry[idx+1]-ry[idx]
            
            if value == rx[idx-1] and value == rx[idx+1]:
                continue
            elif ry[idx] == ry[idx-1] and ry[idx] == ry[idx+1]:
                continue
            elif  (diff_x != 0.0 and rx[idx]-rx[idx-1] == diff_x) and (diff_y != 0.0 and ry[idx]-ry[idx-1] == diff_y):
                continue

            else:
                nrx.append(value)
                nry.append(ry[idx])
    return nrx, nry

def findpath(sx,sy,gx,gy,ox,oy,show_animation, robot_radius):
    """
    A function that uses astarplanner to plan the path out
    Arguments:  starting location, end location, location of obstacles, and robot_radius
    Returns: nrc, nry
    """
    grid_size = 5  # [cm]
    #robot_radius = 12.5 + 5  + 2# [cm]
    
    if show_animation:  # pragma: no cover
        fig, ax = plt.subplots(1,1,figsize=(8,8))
        plt.ylim((-ARENA_WIDTH/2, ARENA_WIDTH/2))
        plt.xlim((-ARENA_WIDTH/2, ARENA_WIDTH/2))
        lims = np.array([-ARENA_WIDTH/2-0.1, ARENA_WIDTH/2+0.1])

        arena_patch = Rectangle((-ARENA_WIDTH/2, -ARENA_WIDTH/2), width = ARENA_WIDTH, height = ARENA_WIDTH, alpha = 0.5, fc = 'gray', ec = 'k', linestyle = '-', linewidth = 2.0)
        red_patch   = Rectangle((ARENA_WIDTH/2-HOME_WIDTH, ARENA_WIDTH/2-HOME_WIDTH), width = HOME_WIDTH, height = HOME_WIDTH, alpha = 0.5, fc = 'r')
        blue_patch   = Rectangle((-ARENA_WIDTH/2, ARENA_WIDTH/2-HOME_WIDTH), width = HOME_WIDTH, height = HOME_WIDTH, alpha =0.5, fc = 'b')
        
        ax.add_patch(arena_patch)
        ax.add_patch(red_patch)
        ax.add_patch(blue_patch)
        ax.set_ylim(lims)
        ax.set_xlim(lims)

        ax.plot(ox, oy, ".k")
        ax.plot(sx, sy, "og")
        ax.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius, move_straight)
    rx, ry = a_star.planning(sx, sy, gx, gy)

    if move_straight:
        nrx, nry = remove_redundant(rx, ry)
    else:
        nrx, nry = get_corners(rx, ry)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.plot(nrx, nry, ".b")
        plt.pause(0.001)
        plt.show()

    return nrx,nry

def get_total_path(current_coordinates,ox,oy,destination,path,a,show_animation=True,robot_radius=19.5):
    """
    A function that sets the starting and desired locations and uses findpath (above function) to plan the path out and add it to the original path
    Arguments: current coordinates, location of obstacles, desired location, origina path and robot_radius
    Returns: path
    """
# start and goal position
    sx = round(current_coordinates[1]*100)  # [cm]
    sy = round(current_coordinates[0]*100)  # [cm]
    gx = round(destination[1]*100)  # [cm]
    gy = round(destination[0]*100)  # [cm]

    nrx, nry = findpath(sx,sy,gx,gy,ox,oy,show_animation,robot_radius)
    nrx.reverse()
    nry.reverse()
    for i in range(len(nrx)):
        new_coordinates= np.array([nry[i]/100,nrx[i]/100])
        path = np.insert(path, a+2+i, new_coordinates, axis = 0)
    
    return path

def find_bearing(relative_vector):
    """
    This function returns the bearing of a vector from North.
    Returns: bearing
    """
    acw_from_x = np.arctan2(relative_vector[1], relative_vector[0]) / deg2rad
    bearing = (-1) * acw_from_x
    if bearing <= 0:
        bearing += 360
    return bearing

def find_intersection(a,b,c,d):
    """"
    Returns the intersection coordinates of a pair of 2D lines.

    Arguments: a, b (two arbitrary points on one line), c, d (two arbitrary points on the other line)
    """
    pz = ((a[1]*b[0]-a[0]*b[1])*(c[1]-d[1])-(a[1]-b[1])*(c[1]*d[0]-c[0]*d[1])) / ((a[1]-b[1])*(c[0]-d[0])-(a[0]-b[0])*(c[1]-d[1]))
    px = ((a[1]*b[0]-a[0]*b[1])*(c[0]-d[0])-(a[0]-b[0])*(c[1]*d[0]-c[0]*d[1])) / ((a[1]-b[1])*(c[0]-d[0])-(a[0]-b[0])*(c[1]-d[1]))
    return px,pz

def check_next_point(current_coords, desired_coords, check_distance, other_colour_blocks):
    """
    This function check if there is an obstacle along the path
    Arguments: current_coords, desired_coords, check_distance, other_colour_blocks
    Returns: block (Boolean, true if there is obstacle)
    """
    #define path step
    path_step = 0.0015
    #find vector of travel
    relative_vector = desired_coords - current_coords
    #normalise to give a direction vector
    direction_vector = relative_vector/check_distance
    #find number of iterations to make for the given step
    n_iteration = int(check_distance / path_step)
    #create list of lambda l_values
    l_values = []
    for i in range(1, n_iteration):
        l_values.append(i * path_step)
    #now iterate through lambda until collides with a block
    block = False
    for l in l_values:
        #find the coordinates on the line at this lambda
        line_coords = current_coords + l * direction_vector
        #now check if it collides with any block
        for block_coords in other_colour_blocks:
            line_block_vector = block_coords - line_coords
            d_line_block = np.linalg.norm(line_block_vector)
            #if it collides with a block, return true
            if d_line_block < pass_distance:
                block = True
                break
        if block:
            break
    return block