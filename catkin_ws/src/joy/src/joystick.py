#!/usr/bin/env python3

from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from inputs import get_gamepad

def main():
    velocity = Twist()
    pubVelocity = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
    pubGripper = rospy.Publisher('/controller',Int16, queue_size=10)
    rospy.init_node('control',anonymous=False) 
    while not rospy.is_shutdown():
        events = get_gamepad()
        for event in events:
            if event.code == "ABS_Y":
                velocity.linear.x = map(event.state)
            if event.code == "ABS_X":
                velocity.angular.z = -map(event.state)
            if event.code == "BTN_PINKIE":
                if event.state == 1:
                    print("SpeedUp")
                    pubGripper.publish(5)
            if event.code == "BTN_BASE2":
                if event.state == 1:
                    print("SpeedDown")
                    pubGripper.publish(6)
            if event.code == "BTN_TRIGGER":
                if event.state == 1:
                    print("ArmUp")
                    pubGripper.publish(7)
                else:
                    print("Stop lifting")
                    pubGripper.publish(9)
            if event.code == "BTN_THUMB2":
                if event.state == 1:
                    print("ArmDown")
                    pubGripper.publish(8)
                else:
                    print("Stop lifting")
                    pubGripper.publish(9)
            if event.code == "BTN_THUMB":
                if event.state == 1:
                    print("ReleaseMine")
                    pubGripper.publish(10)
            if event.code == "BTN_TOP":
                if event.state == 1:
                    print("HoldMine")
                    pubGripper.publish(11)

            print(str(velocity))
            pubVelocity.publish(velocity)

def map(x, in_min=0, in_max=255, out_min=1, out_max=-1):
    if x in range(120,135):
       return 0
    else:
        return float(out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min))
                
if __name__=="__main__":            
    main()