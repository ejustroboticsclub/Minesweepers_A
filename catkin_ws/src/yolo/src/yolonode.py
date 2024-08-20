#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

def read_detection_status():
    try:
        with open("/home/ahmed/catkin_ws/src/yolo/src/detection_status.txt", "r") as f:
            status = f.read().strip().lower() == "true"
            return status
    except FileNotFoundError:
        rospy.logwarn("detection_status.txt not found, defaulting to False")
        return False

def publish_detection_status():
    # Initialize the ROS node
    rospy.init_node('camera_detection_node', anonymous=False)
    
    # Create a publisher for the topic '/camera_detection'
    detection_pub = rospy.Publisher('/camera_detection', Bool, queue_size=10)
    
    # Define the rate at which to publish the messages (in Hz)
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        # Read the current detection status from the file
        detection_status = read_detection_status()
        
        # Create a Bool message
        detection_msg = Bool()
        detection_msg.data = detection_status
        
        # Publish the message
        detection_pub.publish(detection_msg)
        
        # Log the published message
        rospy.loginfo(f"Published detection status: {detection_status}")
        
        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_detection_status()
    except rospy.ROSInterruptException:
        pass
