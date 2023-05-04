import matplotlib.pyplot as plt
import numpy as np
coordgraph = open("../data/XYfiles/hut.dot", "r")


xarr = []
yarr = []


fig = plt.figure()
ax = fig.add_subplot(111)
#code to plot the xarr and yarr with a 0.1 second delay
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

for line in coordgraph:
    x,y = line.split(",")
    xarr.append(int(x))
    yarr.append(int(y))

    # plt.plot(xarr, yarr, 'ro')
    # plt.show()
    # plt.pause(0.01)
    # plt.close()
    # plt.clf()
    # plt.axis([-1, 1, -1, 1])
    # plt.grid(True)
    # plt.draw()

# plt.show()


# x_diff = []
# y_diff = []

# for i in range(len(xarr)-1):
#     x_diff.append(xarr[i+1] - xarr[i])
#     y_diff.append(yarr[i+1] - yarr[i])

# timeline = np.linspace(0, 1, len(x_diff))
# plt.plot(timeline,x_diff)
# plt.plot(timeline,y_diff)
# plt.show()


# create 1000 equally spaced points between -10 and 10
# diff = 10
# x = np.linspace(-diff/2, diff/2, 100)
# y = -x**2 
# y = -(y-np.min(y))/(np.max(y)-np.min(y)) * 0.05 - 0.01


_ = getCircleXY()

# fig, ax = plt.subplots()
# ax.plot(x, y)
# # plt.axis('equal')
# plt.show()