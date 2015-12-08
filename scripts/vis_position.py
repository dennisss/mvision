#!/usr/bin/python2

# Given a file containing [w x y z] quaternions on each line, this script will show the axes change in a figure

import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import time


if len(sys.argv) != 2:
    print("Usage ./vis_position out.txt")



data = np.loadtxt(sys.argv[1], dtype=np.float)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d', aspect=1)


ax.plot(data[:,4], data[:,5], data[:,6])


ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()
