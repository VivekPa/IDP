import numpy as np
import os
import csv

list_of_blocks = [[1,1],[2,2]]

dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, './paths/horizontal_traverse.txt')

red_base    = [1,1]
red_standby = [1,0.7]
blue_base   = [1,-1]
blue_standby = [1,-0.7]
home        = red_base

path = np.array([home,home])
with open(filename, 'r') as csvfile:
    csvreader = csv.reader(csvfile,  delimiter = ',')
    for row in csvreader:
        for j in range(len(row)):
            row[j] = float(row[j])
        point = (np.array(row))
        list_point = list(point)
        path = np.vstack([path, point])

path = np.append(path, np.array(list_of_blocks), axis = 0)
print(path)