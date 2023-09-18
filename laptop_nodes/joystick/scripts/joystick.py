#!/usr/bin/env python

from __future__ import print_function
import rospy
from std_msgs.msg import Int16
from inputs import get_gamepad

def main():
    pub = rospy.Publisher('/controller',Int16, queue_size=10)
    rospy.init_node('control',anonymous=True) 
    while not rospy.is_shutdown():
        events = get_gamepad()
        for event in events:
            if event.code == "ABS_Y":
                if event.state == 0:
                    print("Forward")
                    pub.publish(1)
                elif event.state == 255:
                    print("Backward")
                    pub.publish(2)
                else:
                    print("Stop")
                    pub.publish(0)
            if event.code == "ABS_X":
                if event.state == 255:
                    print("Right")
                    pub.publish(3)
                elif event.state == 0:
                    print("Left")
                    pub.publish(4)
                else:
                    print("Stop")
                    pub.publish(0)
            if event.code == "BTN_PINKIE":
                if event.state == 1:
                    print("SpeedUp")
                    pub.publish(5)
            if event.code == "BTN_BASE2":
                if event.state == 1:
                    print("SpeedDown")
                    pub.publish(6)
            if event.code == "BTN_TRIGGER":
                if event.state == 1:
                    print("ArmUp")
                    pub.publish(7)
                else:
                    print("Stop lifting")
                    pub.publish(9)
            if event.code == "BTN_THUMB2":
                if event.state == 1:
                    print("ArmDown")
                    pub.publish(8)
                else:
                    print("Stop lifting")
                    pub.publish(9)
            if event.code == "BTN_THUMB":
                if event.state == 1:
                    print("ReleaseMine")
                    pub.publish(10)
            if event.code == "BTN_TOP":
                if event.state == 1:
                    print("HoldMine")
                    pub.publish(11)

                
if __name__=="__main__":            
    main()