#!/usr/bin/python2

# Given a file containing [w x y z] quaternions on each line, this script will show the axes change in a figure

import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import time


def quat2mat(q):
    qr, qi, qj, qk = q
    return np.matrix([
        [(1-2*qj*qj-2*qk*qk), 2*(qi*qj-qk*qr), 2*(qi*qk+qj*qr)],
        [2*(qi*qj+qk*qr), (1-2*qi*qi-2*qk*qk), 2*(qj*qk-qi*qr)],
        [2*(qi*qk-qj*qr), 2*(qj*qk+qi*qr), (1-2*qi*qi-2*qj*qj)]
    ])



#import cv2


if len(sys.argv) != 2:
    print("Usage ./vis_axes orientation_data.txt")



data = np.loadtxt(sys.argv[1], dtype=np.float)
#cap = cv2.VideoCapture('data/1447955692095000000.mp4')

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d', aspect=1)
#ax = fig.add_subplot(121, projection='3d', aspect=1)
#ax_img = fig.add_subplot(122)
#img_artist = ax_img.imshow(np.zeros((360, 480, 3)))


# Create lines for x, y and z
#lines = ax.plot([0,0],[0,0],[0,1],'r-') + ax.plot([0,0],[0,0],[0,1],'g-') + ax.plot([0,0],[0,0],[0,1],'b-')

ps =[
    np.matrix([[1],[0],[0]]),
    np.matrix([[0],[1],[0]]),
    np.matrix([[0],[0],[1]])
]
pcolors = ['r', 'g', 'b']

cam_pts = np.matrix([
    [0,0,0],
    [-1,-1,-1],
    [-1,1,-1],
    [0,0,0],
    [-1,1,-1],
    [1,1,-1],
    [0,0,0],
    [1,1,-1],
    [1,-1,-1],
    [0,0,0],
    [1,-1,-1],
    [-1,-1,-1]
])*0.5


plt.ion()

for i in range(0, data.shape[0]):
    print(i)

    #ret, frame = cap.read()
    #frame = cv2.resize(frame, (480, 360))
    #im = np.empty_like(frame)
    #im[:,:,0] = frame[:,:,2]
    #im[:,:,1] = frame[:,:,1]
    #im[:,:,2] = frame[:,:,0]
    #img_artist.set_data(im)


    q = data[i, 0:4]
    Rt = quat2mat(q).transpose()

    ax.clear()

    for j in range(0, len(ps)):
        p = ps[j]

        px = np.dot(Rt, p)

        ax.plot([0, px[0,0]], [0, px[1,0]], [0, px[2,0]], pcolors[j] + '-')

    cam = np.dot(Rt, cam_pts.transpose())
    ax.plot(cam[0,:].tolist()[0], cam[1,:].tolist()[0], cam[2,:].tolist()[0], 'b-')


    #fig.canvas.draw()

    ax.set_xlabel('X')
    ax.set_xlim(-1, 1)
    ax.set_ylabel('Y')
    ax.set_ylim(-1, 1)
    ax.set_zlabel('Z')
    ax.set_zlim(-1, 1)

    #plt.draw()
    plt.pause(0.0001)


# http://stackoverflow.com/questions/4098131/how-to-update-a-plot-in-matplotlib
