#!/usr/bin/env python
import matplotlib.pyplot as plt
from matplotlib import colors
import numpy as np
import string
from matplotlib.animation import FuncAnimation
import rospy
from geometry_msgs.msg import Pose2D
import math

x=None
y=None
z=None

def mapper(msg):
    global x
    global y
    global z
    x= int(math.floor(msg.x))
    y=int(math.floor(msg.y))
    z=int(math.floor(msg.theta))



    



if __name__ == '__main__':

    
    n = 20
    data = np.random.rand(n, n) * np.nan

    # set some values to 0, 1, 2
    data[0, 0] = 0
    data[9, 9] = 1
    data[16, 16] = 2


    cmap = colors.ListedColormap(['red', 'blue','green'])

    fig, ax = plt.subplots()
    im = ax.imshow(data, cmap=cmap ,extent=[0, n, 0, n], zorder=2)

    # draw gridlines
    ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=2)


    alphabets = string.ascii_uppercase[:n-1]
    ticklabels = [0] + list(alphabets)
    ax.set_xticks(np.arange(0, n, 1), minor=False)
    ax.set_xticklabels(ticklabels, minor=False)
    # ax.invert_xaxis()

    ax.set_yticks(np.arange(0, n, 1), minor=False);
    ax.invert_yaxis()
    ax.set_yticklabels(np.arange(20, 0, -1), minor=False)
    data = np.random.rand(n, n) * np.nan
    def update(frame):
    # update data for each frame
    # cmap = colors.ListedColormap(['red', 'blue','green'])
        data[x, y] = z

        im.set_data(data)
    ani = FuncAnimation(fig, update)
    rospy.init_node("plotter")
    rospy.Subscriber("navigation", Pose2D, mapper)
    plt.show()
                    




