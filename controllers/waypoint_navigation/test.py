import numpy as np
import csv

path = np.array([[0.0,  0.0],[1.0,  1.0]])
a = np.array([1.0, 2.0, 3.0])

with open("paths/horizontal_traverse.txt", 'r') as csvfile:
    csvreader = csv.reader(csvfile,  delimiter = ',')
    for row in csvreader:
        for i in range(len(row)):
            row[i] = float(row[i])
        point = (np.array(row))
        path = np.vstack([path, point])

# print(path[0].shape)
# print(np.array([[1,1],[2,2],[3,3]]).shape[0])

subpath = np.array([[1,1],[2,2],[3,3]])
# print(subpath[0])
# print(subpath)
# print(np.delete(a,-1, axis=0))
# print(np.vstack((path, [5,5])))
res = np.insert(path, 1, [5,5], axis=0)
print(res)