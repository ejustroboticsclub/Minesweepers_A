#!/usr/bin/env python

import rospy
from math import sin, cos  # , pi
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion, Pose2D
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int16
import numpy as np


class DiffTf:
    def __init__(self):
        """
        Initialize the DiffTf node.

        Sets up the ROS node, parameters, subscribers, publishers, and internal variables.

        Parameters:
            None

        Subscribes to:
            - "left_ticks"  (Int16) : Left wheel encoder ticks.
            - "right_ticks" (Int16) : Right wheel encoder ticks.
            - "/imu_data"   (Imu)   : IMU data for orientation.

        Publishes:
            - "odom"            (Odometry)  : Odometry information including position and velocity.
            - "/robot_2d"       (Pose2D)    : 2D pose of the robot.
            - "/encoder_dist"   (Pose2D)    : Distance traveled as reported by encoders.
        """

        rospy.init_node("diff_tf")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)

        #### parameters #######
        self.rate = rospy.get_param(
            "~rate", 10.0
        )  # the rate at which to publish the transform
        self.ticks_meter_r = float(
            rospy.get_param("ticks_meter", 2255)
        )  # The number of wheel encoder ticks per meter of travel
        self.ticks_meter_l = float(
            rospy.get_param("ticks_meter", 3800)
        )  # The number of wheel encoder ticks per meter of travel
        self.base_width = float(
            rospy.get_param("~base_width", 0.565)
        )  # The wheel base width in meters

        self.base_frame_id = rospy.get_param(
            "~base_frame_id", "base_link"
        )  # the name of the base frame of the robot
        self.odom_frame_id = rospy.get_param(
            "~odom_frame_id", "odom"
        )  # the name of the odometry reference frame

        self.encoder_min = rospy.get_param("encoder_min", -2147483648)
        self.encoder_max = rospy.get_param("encoder_max", 2147483648)
        self.encoder_low_wrap = rospy.get_param(
            "wheel_low_wrap",
            (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min,
        )
        self.encoder_high_wrap = rospy.get_param(
            "wheel_high_wrap",
            (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min,
        )

        self.t_delta = rospy.Duration(1.0 / self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        self.mine_pose_pub = rospy.Publisher("/mine_pose", Pose2D, queue_size=10)

        # internal data
        self.enc_left = None  # wheel encoder readings
        self.enc_right = None
        self.yaw_prev = None
        self.left = 0  # actual values coming back from robot
        self.right = 0
        self.lmult = 0
        self.rmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0.6  # position in xy plane
        self.y = 0
        self.th = 0
        self.yaw = 0
        self.dx = 0  # speeds in x/rotation
        self.dr = 0
        self.then = rospy.Time.now()
        self.l = 1.50
        # subscriptions
        rospy.Subscriber("left_ticks", Int16, self.lwheelCallback, queue_size=1)
        rospy.Subscriber("right_ticks", Int16, self.rwheelCallback, queue_size=1)
        rospy.Subscriber("/imu/data", Imu, self.imu_callback)

        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.pose_pub = rospy.Publisher("/robot_2d", Pose2D, queue_size=10)
        self.encoder_dist_pub = rospy.Publisher("/encoder_dist", Pose2D, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()

        # variables for mine detection
        self.detection = 0
        self.location = 0
        self.prev_x = 0
        self.prev_y = 0

    def spin(self):
        """
        Main loop for the DiffTf node.

        Continuously updates and publishes odometry and pose information at the specified rate.

        Parameters:
            None

        Publishes:
            - Odometry and Pose2D messages.
        """

        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

    def imu_callback(self, imu_msg):
        """
        Callback function for IMU data.

        Converts IMU orientation quaternion to Euler angles and updates the yaw angle.

        Parameters:
            imu_msg (Imu): IMU message containing orientation data.

        Updates:
            - self.yaw: The yaw angle derived from IMU data.
        """

        self.orientation_q = imu_msg.orientation
        self.orientation_list = [
            self.orientation_q.x,
            self.orientation_q.y,
            self.orientation_q.z,
            self.orientation_q.w,
        ]
        (roll, pitch, self.yaw) = euler_from_quaternion(self.orientation_list)

    def update(self):
        """
        Update the robot's odometry and pose information.

        Calculates the distance traveled, velocity, and updates the robot's position
        based on encoder readings and yaw angle. Publishes updated odometry and pose information.

        Parameters:
            None

        Publishes:
            - Odometry  : Updated position and velocity of the robot.
            - Pose2D    : 2D pose of the robot.
        """

        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()

            # calculate odometry
            if self.enc_left is None:
                self.yaw_prev = 0
                delta_yaw = 0
                d_left = 0
                d_right = 0
            else:
                d_left = (self.left - self.enc_left) / self.ticks_meter_l
                d_right = (self.right - self.enc_right) / self.ticks_meter_r
                delta_yaw = self.yaw - self.yaw_prev
            self.enc_left = self.left
            self.enc_right = self.right
            self.yaw_prev = self.yaw

            # distance traveled is the average of the two wheels
            d = d_right
            th = delta_yaw

            # publish the odom information
            self.enc_pub = Pose2D()
            self.enc_pub.x = d_left
            self.enc_pub.y = d_right
            self.enc_pub.theta = d
            self.encoder_dist_pub.publish(self.enc_pub)

            # calculate velocities
            self.dx = d / elapsed
            self.dr = th / elapsed

            if d != 0:
                # calculate distance traveled in x and y
                x = cos(th) * d
                y = sin(th) * d
                # calculate the final position of the robot
                self.x += cos(self.th) * x - sin(self.th) * y
                self.y += sin(self.th) * x + cos(self.th) * y
            if th != 0:
                self.th += th

            # pose2d publish
            self.robo_2d = Pose2D()
            self.robo_2d.x = self.x
            self.robo_2d.y = self.y
            self.robo_2d.theta = self.th
            self.pose_pub.publish(self.robo_2d)

            # publish the odom information
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.th / 2)
            quaternion.w = cos(self.th / 2)
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id,
            )

            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = self.dx
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = self.dr
            self.odomPub.publish(odom)
            rospy.loginfo(
                "Odometry updated: x=%f, y=%f, theta=%f", self.x, self.y, self.th
            )

    def lwheelCallback(self, msg: Int16):
        """
        Callback function for left wheel encoder ticks.

        Updates the left wheel encoder reading and corrects for encoder wrap-around.

        Parameters:
            msg (Int16): Message containing the left wheel encoder ticks.

        Updates:
            - self.left: Corrected left wheel encoder reading.
        """

        enc = msg.data
        if enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap:
            self.lmult += 1

        if enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap:
            self.lmult -= 1

        self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min))
        self.prev_lencoder = enc
        rospy.loginfo("Left Wheel Encoder data: left_ticks=%d", self.left)

    def rwheelCallback(self, msg: Int16):
        """
        Callback function for right wheel encoder ticks.

        Updates the right wheel encoder reading and corrects for encoder wrap-around.

        Parameters:
            msg (Int16) : Message containing the right wheel encoder ticks.

        Updates:
            - self.right: Corrected right wheel encoder reading.
        """

        enc = msg.data
        if enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap:
            self.rmult += 1

        if enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap:
            self.rmult -= 1

        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))
        self.prev_rencoder = enc
        rospy.loginfo("Right Wheel Encoder data: right_ticks=%d", self.right)

    def mine_pose(self, msg):
        self.mine_position = Pose2D()
        self.location = msg.data

        if self.detection == 1:
            print("detection: ", self.detection)
            if self.location == 1:
                print("location:", self.location)
                self.mine_position.x = self.x + self.l * np.cos(self.yaw)
                self.mine_position.y = self.y + self.l * np.sin(self.yaw)

                if (abs(self.mine_position.x - self.prev_x) > 0.8) or (
                    abs(self.mine_position.y - self.prev_y) > 0.8
                ):
                    print("location 1 and pose >0.6")

                    self.prev_x = self.mine_position.x
                    self.prev_y = self.mine_position.y
                    self.mine_position.theta = 1
                    self.mine_pose_pub.publish(self.mine_position)
            elif self.location == 2:
                print("location:", self.location)
                self.mine_position.x = self.x + self.l * np.cos(self.yaw)
                self.mine_position.y = self.y + self.l * np.sin(self.yaw)

                if (abs(self.mine_position.x - self.prev_x) > 0.8) or (
                    abs(self.mine_position.y - self.prev_y) > 0.8
                ):
                    print("location 2 and pose >0.6")

                    self.prev_x = self.mine_position.x
                    self.prev_y = self.mine_position.y
                    self.mine_position.theta = 2
                    self.mine_pose_pub.publish(self.mine_position)


if __name__ == "__main__":
    try:
        diff_tf = DiffTf()
        diff_tf.spin()
    except rospy.ROSInterruptException:
        pass
