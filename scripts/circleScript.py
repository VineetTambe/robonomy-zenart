import numpy as np
import matplotlib.pyplot as plt
import datetime
from mpl_toolkits.mplot3d import Axes3D


def getClockXY(r = 0.05):
    deltas = []
    # deltas.append(np.array([0, 0, 0]))
    theta = np.linspace(0, np.pi * 2, 100)
    
    x = r * np.sin(theta) 
    y = r * np.cos(theta) 
    z = np.zeros_like(x)
    path = list(zip(x, y, z))
    for p in path:
        deltas.append(p)

    # following lines for the clock arms

    xi, yi, zi = np.array(deltas[-1]) - 0.001
    xf, yf, zf = 0.0, 0.0, zi - 0.05

    moveUpx = np.linspace(xi, xf, 10)
    moveUpy = np.linspace(yi, yf, 10)
    moveUpz = np.linspace(zi, zf, 10)
    print(np.vstack([moveUpx, moveUpy, moveUpz]).T.shape)
    deltas += list(np.vstack([moveUpx, moveUpy, moveUpz]).T)
    shape = moveUpx.shape
    deltas += list(np.vstack([np.random.randn(*shape) * 1e-10, np.random.randn(*shape) * 1e-10, moveUpz[::-1] + 0.001]).T)

    now = datetime.datetime.now()

    hour = now.hour % 12
    minute = now.minute % 60
    hourAngle = (2 * np.pi * hour / 12  + (minute / 60) * (2 * np.pi/12)) - np.pi/2
    # for i in np.linspace(0, r - 0.03, 10): 
    #     arm1x = i * np.cos(hourAngle)
    #     arm1y = i * np.sin(hourAngle)
    #     arm1z = 0.0
    #     deltas.append([arm1x, arm1y, arm1z])


    # # trajectory that lifts and goes to center
    xi, yi, zi = 0.0, 0.0, zi - 0.05
    xf, yf, zf = np.array(deltas[-1]) - 0.001



    moveUpx = np.linspace(xi, xf, 10)
    moveUpy = np.linspace(yi, yf, 10)
    moveUpz = np.linspace(zi, zf, 10)
    print(np.vstack([moveUpx, moveUpy, moveUpz]).T.shape)
    # deltas += list(np.vstack([moveUpx, moveUpy, moveUpz]).T)
    # shape = moveUpx.shape
    # deltas += list(np.vstack([np.random.randn(*shape) * 1e-10, np.random.randn(*shape) * 1e-10, moveUpz[::-1] + 0.001]).T)



    # print(hour, minute)


    # minuteAngle = (minute * 2 * np.pi / 60) - np.pi/2
    # # minuteAngle = 0

    # for i in np.linspace(0, r - 0.01, 10): 
    #     arm1x = i * np.cos(minuteAngle)
    #     arm1y = i * np.sin(minuteAngle)
    #     arm1z = 0.0
    #     deltas.append([arm1x, arm1y, arm1z])


    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    p = np.vstack(deltas)
    ax.scatter(p[:, 0], p[:, 1], p[:, 2])
    # plt.axis('equal')
    plt.show()
    return p

if __name__ =="__main__":
    getClockXY()