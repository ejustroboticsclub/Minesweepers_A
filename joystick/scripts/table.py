import string
import rospy
from geometry_msgs.msg import Pose2D
import pandas as pd

surface_mines = []
buried_mines =[]
def node_init():
    rospy.init_node("table_rep",anonymous=True)
    rospy.Subscriber("/mine_pose", Pose2D, table)
    rospy.spin()


def table(data):
    x = data.x
    y = data.y
    z=data.theta
    if (z==1):
        item = [x+1,string.ascii_uppercase[y]]
        if item not in surface_mines:
            surface_mines.append(item)
    elif (z==2):
        item = [x+1,string.ascii_uppercase[y]]
        if item not in buried_mines:
            buried_mines.append(item)
    df = pd.DataFrame (surface_mines, columns = ['x','y'])
    filepath = 'surface_mines.xlsx'
    df.to_excel(filepath, index=False)
    df = pd.DataFrame (buried_mines, columns = ['x','y'])
    filepath = 'buried_mines.xlsx'
    df.to_excel(filepath, index=False)

if __name__ == '__main__':
    node_init()


    