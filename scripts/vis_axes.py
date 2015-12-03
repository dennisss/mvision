#!/usr/bin/python2

# Given a file containing [w x y z] quaternions on each line, this script will show the axes change in a figure

import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import time

import Quaternion
from Quaternion import Quat


if len(sys.argv) != 2:
    print("Usage ./vis_axes orientation_data.txt")



data = np.loadtxt(sys.argv[1], dtype=np.float)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d', aspect=1)


# Create lines for x, y and z
#lines = ax.plot([0,0],[0,0],[0,1],'r-') + ax.plot([0,0],[0,0],[0,1],'g-') + ax.plot([0,0],[0,0],[0,1],'b-')

ps =[
    np.matrix([[1],[0],[0]]),
    np.matrix([[0],[1],[0]]),
    np.matrix([[0],[0],[1]])
]
pcolors = ['r', 'g', 'b']


plt.ion()

for i in range(0, data.shape[0], 10):
    print(i)

    q = data[i,:]
    Rt = Quat(Quaternion.normalize(q)).transform

    ax.clear()

    for j in range(0, len(ps)):
        p = ps[j]

        px = np.dot(Rt, p)

        ax.plot([0, px[0,0]], [0, px[1,0]], [0, px[2,0]], pcolors[j] + '-')

    #fig.canvas.draw()

    ax.set_xlabel('X')
    ax.set_xlim(-1, 1)
    ax.set_ylabel('Y')
    ax.set_ylim(-1, 1)
    ax.set_zlabel('Z')
    ax.set_zlim(-1, 1)

    plt.pause(0.0001)


# http://stackoverflow.com/questions/4098131/how-to-update-a-plot-in-matplotlib
