from utils.astarplanner import AStarPlanner
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Rectangle
import numpy as np

move_straight = False
ARENA_WIDTH     = 240 #[cm]
HOME_WIDTH      = 40    #[cm]

def remove_redundant(xc, yc): #for straight movement
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
    nrx, nry = [], []
    for idx,value in enumerate(rx):
        if idx == len(rx)-1:
            nrx.append(value)
            nry.append(ry[idx])
            break
        else:
            diff_x = rx[idx+1]-rx[idx]
            diff_y = ry[idx+1]-ry[idx]
            #print(diff_x,diff_y)
            if value == rx[idx-1] and value == rx[idx+1]:
                #print(value,ry[idx])
                continue
            elif ry[idx] == ry[idx-1] and ry[idx] == ry[idx+1]:
                #print(value,ry[idx])
                continue
            elif  (diff_x != 0.0 and rx[idx]-rx[idx-1] == diff_x) and (diff_y != 0.0 and ry[idx]-ry[idx-1] == diff_y):
                continue

            else:
                nrx.append(value)
                nry.append(ry[idx])
    return nrx, nry

def findpath(sx,sy,gx,gy,ox,oy,show_animation):
    
    grid_size = 5  # [cm]
    robot_radius = 12.5 + 5  + 2# [m]
    
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

    print(rx, ry)
    print(nrx, nry)
    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.plot(nrx, nry, ".b")
        plt.pause(0.001)
        plt.show()

    return nrx,nry

def get_total_path(current_coordinates,ox,oy,destination,path,a,show_animation=True):
# start and goal position
    sx = round(current_coordinates[1]*100)  # [cm]
    sy = round(current_coordinates[0]*100)  # [cm]
    gx = round(destination[1]*100)  # [cm]
    gy = round(destination[0]*100)  # [cm]

    nrx, nry = findpath(sx,sy,gx,gy,ox,oy,show_animation)
    nrx.reverse()
    nry.reverse()
    for i in range(len(nrx)):
        new_coordinates= np.array([nry[i]/100,nrx[i]/100])
        path = np.insert(path, a+2+i, new_coordinates, axis = 0)
    
    return path