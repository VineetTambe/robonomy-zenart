import numpy as np
import matplotlib.pyplot as plt
import datetime
from mpl_toolkits.mplot3d import Axes3D


def euclidianDist(x1,y1,x2,y2):
    return np.sqrt((x2-x1)**2 + (y2-y1)**2)

def getCircleXY(r = 0.05):
    deltas = []
    # deltas.append(np.array([0, 0, 0]))
    theta = np.linspace(0, np.pi * 2, 100)
    
    x = r * np.sin(theta) 
    y = r * np.cos(theta) 
    z = np.zeros_like(x)
    path = list(zip(x, y, z))
    for p in path:
        deltas.append(p)
    p = np.vstack(deltas)
    p[:,2]=0
    print(p)
    plt.scatter(p[:, 0], p[:, 1])
    plt.show()
    return p

def getTraj(filepath, x_max, y_max, x_min, y_min):
    coordgraph = open(filepath, "r")


    dist_threshold = 3.0

    z_pick_height = 0.02
    z_safety_offset = 0.0
    z_height = 0.0

    xarr = []
    yarr = []
    x_final = []
    y_final = []
    z_final = []

    for line in coordgraph:
        x,y = line.split(",")
        xarr.append(int(x))
        yarr.append(-int(y))

    # flag = []

    x_diff = []
    y_diff = []
    z_diff = []

    for i in range(0,len(xarr)-1, 3):

        dist = euclidianDist(xarr[i],yarr[i],xarr[i+1],yarr[i+1])
        if dist > dist_threshold:
            x_interpolate = np.linspace(xarr[i],xarr[i+1],50)
            y_interpolate = np.linspace(yarr[i],yarr[i+1],50)

            x = np.linspace(-dist/2, dist/2, 50)
            y = -x**2 
            z_interpolate = -(y-np.min(y))/(np.max(y)-np.min(y)) * z_pick_height - z_safety_offset
            x_final += list(x_interpolate)
            y_final += list(y_interpolate)
            z_final += list(z_interpolate)
        else:
            x_final.append(xarr[i])
            y_final.append(yarr[i])
            z_final.append(z_height)

    deltas = []
    
    x_scale = (x_max - x_min) / (np.max(x_final) - np.min(x_final))
    x_final = [(x * x_scale) + x_min for x in x_final]
    y_scale = (y_max - y_min) / (np.max(y_final) - np.min(y_final))
    y_final = [(y * y_scale) + y_min for y in y_final]

    path = list(zip(x_final, y_final, z_final))
    for p in path:
        deltas.append(p)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    p = np.vstack(deltas)

    ax.scatter(p[:, 0], p[:, 1], p[:, 2])
    
    # ax.scatter(x_diff, y_diff, z_diff, c='r', marker='o')
    # print(x_diff)
    # plt.axis('equal')
    # plt.show()
    return p

if __name__ =="__main__":
    getTraj("../data/XYfiles/hut.dot", 0.50,0.50,-0.0,-0.00)


#172.26.162.67