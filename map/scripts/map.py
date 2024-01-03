#!/usr/bin/env python
import matplotlib.pyplot as plt
from matplotlib import colors
import numpy as np
import string
from matplotlib.animation import FuncAnimation
import rospy
from geometry_msgs.msg import Pose2D
import pandas as pd
import math

x=None
y=None
z=None
z_prev = None

def mapper(msg):
    global x
    global y
    global z

    x= int(abs(msg.x))
    if (x>=10):
        x=9
    elif (x<0):
        x=0
    y=int(abs(msg.y))
    if (y>=10):
        y=9
    elif (y<0):
        y=0
    z = int(msg.theta)



    # if (z==1):
    #     item = [x+1,string.ascii_uppercase[y]]
    #     if item not in surface_mines:
    #         surface_mines.append(item)
    # elif (z==2):
    #     item = [x+1,string.ascii_uppercase[y]]
    #     if item not in buried_mines:
    #         buried_mines.append(item)
    # df = pd.DataFrame (surface_mines, columns = ['x','y'])
    # filepath = 'surface_mines.xlsx'
    # df.to_excel(filepath, index=False)
    # df = pd.DataFrame (buried_mines, columns = ['x','y'])
    # filepath = 'buried_mines.xlsx'
    # df.to_excel(filepath, index=False)



    



if __name__ == '__main__':

    # surface_mines = []
    # buried_mines =[]
    # i_surf=0
    # ib=0
    n = 10
    data = np.random.rand(n, n) * np.nan

    # set some values to 0, 1, 2
    data[0, 0] = 2
    data[9, 9] = 1
    data[5, 5] = 0
    


    cmap = colors.ListedColormap(['green','red', 'blue'])

    fig, ax = plt.subplots()
    im = ax.imshow(data, cmap=cmap ,extent=[0, n, 0, n], zorder=2)

    # draw gridlines
    ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=2)


    alphabets = string.ascii_uppercase[:n]
    ticklabels = [0] + list(alphabets)
    ax.set_xticks(range(n + 1), minor=False)
    ax.set_xticklabels(ticklabels, minor=False)
    # ax.invert_xaxis()

    ax.set_yticks(range(n + 1), minor=False)
    ax.invert_yaxis()
    ax.set_yticklabels(np.arange(n+1, 0, -1), minor=False)
    data = np.random.rand(n, n) * np.nan

    def update(frame):
    # update data for each frame
    # cmap = colors.ListedColormap(['red', 'blue','green'])
        data[x,y] = z
        im.set_data(data)

    ani = FuncAnimation(fig, update)
    rospy.init_node("plotter")
    rospy.Subscriber("/mine_pose", Pose2D, mapper)
    plt.show()
                    




