#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Pose2D, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from tf2_ros import TransformBroadcaster

from math import sin, cos
import numpy as np
from tf_transformations import euler_from_quaternion


class DiffTf(Node):
    def __init__(self):
        super().__init__('diff_tf')
        self.get_logger().info("DiffTf node started")

        # --- Parameters ---
        self.declare_parameter('rate', 10.0)
        self.rate = self.get_parameter('rate').value

        self.declare_parameter('ticks_meter_r', 2255.0)
        self.ticks_meter_r = self.get_parameter('ticks_meter_r').value

        self.declare_parameter('ticks_meter_l', 3800.0)
        self.ticks_meter_l = self.get_parameter('ticks_meter_l').value

        self.declare_parameter('base_width', 0.55)
        self.base_width = self.get_parameter('base_width').value

        self.declare_parameter('base_frame_id', 'base_link')
        self.base_frame_id = self.get_parameter('base_frame_id').value

        self.declare_parameter('odom_frame_id', 'odom')
        self.odom_frame_id = self.get_parameter('odom_frame_id').value

        # Encoder wrap parameters
        self.encoder_min = -2147483648
        self.encoder_max = 2147483648
        self.encoder_low_wrap = (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min
        self.encoder_high_wrap = (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min

        # --- Internal variables ---
        self.enc_left = None
        self.enc_right = None
        self.yaw_prev = 0
        self.left = 0
        self.right = 0
        self.lmult = 0
        self.rmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.yaw = 0.0
        self.dx = 0.0
        self.dr = 0.0
        self.l = 0.55  # distance from robot center to sensor

        self.prev_x = 0
        self.prev_y = 0
        self.detection = 0
        self.location = 0

        # --- Publishers ---
        qos = QoSProfile(depth=10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos)
        self.pose_pub = self.create_publisher(Pose2D, 'robot_2d', qos)
        self.encoder_dist_pub = self.create_publisher(Pose2D, 'encoder_dist', qos)
        self.mine_pose_pub = self.create_publisher(Pose2D, 'mine_pose', qos)

        # --- Subscribers ---
        self.create_subscription(Int16, 'left_ticks', self.lwheel_callback, qos)
        self.create_subscription(Int16, 'right_ticks', self.rwheel_callback, qos)
        self.create_subscription(Int16, 'mine_detector', self.mine_pose, qos)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, qos)

        # --- TF Broadcaster ---
        self.odom_broadcaster = TransformBroadcaster(self)

        # --- Timer ---
        self.timer_period = 1.0 / self.rate
        self.timer = self.create_timer(self.timer_period, self.update)

    def imu_callback(self, imu_msg: Imu):
        q = imu_msg.orientation
        (roll, pitch, self.yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def lwheel_callback(self, msg: Int16):
        enc = msg.data
        if enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap:
            self.lmult += 1
        if enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap:
            self.lmult -= 1
        self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min))
        self.prev_lencoder = enc
        self.get_logger().info(f"Left ticks: {self.left}")

    def rwheel_callback(self, msg: Int16):
        enc = msg.data
        if enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap:
            self.rmult += 1
        if enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap:
            self.rmult -= 1
        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))
        self.prev_rencoder = enc
        self.get_logger().info(f"Right ticks: {self.right}")

    def mine_pose(self, msg: Int16):
        self.location = msg.data
        if self.detection == 1:
            mine_pos = Pose2D()
            mine_pos.x = self.x + self.l * cos(self.yaw)
            mine_pos.y = self.y + self.l * sin(self.yaw)
            if (abs(mine_pos.x - self.prev_x) > 0.8) or (abs(mine_pos.y - self.prev_y) > 0.8):
                self.prev_x = mine_pos.x
                self.prev_y = mine_pos.y
                mine_pos.theta = self.location
                self.mine_pose_pub.publish(mine_pos)

    def update(self):
        now = self.get_clock().now()
        elapsed = self.timer_period

        # --- Odometry calculation ---
        if self.enc_left is None:
            delta_yaw = 0.0
            d_left = 0.0
            d_right = 0.0
        else:
            d_left = (self.left - self.enc_left) / self.ticks_meter_l
            d_right = (self.right - self.enc_right) / self.ticks_meter_r
            delta_yaw = self.yaw - self.yaw_prev

        self.enc_left = self.left
        self.enc_right = self.right
        self.yaw_prev = self.yaw

        d = (d_left + d_right) / 2.0
        th = delta_yaw

        if d != 0:
            dx = cos(th) * d
            dy = sin(th) * d
            self.x += cos(self.th) * dx - sin(self.th) * dy
            self.y += sin(self.th) * dx + cos(self.th) * dy
        if th != 0:
            self.th += th

        self.dx = d / elapsed
        self.dr = th / elapsed

        # --- Publish Pose2D ---
        pose2d = Pose2D()
        pose2d.x = self.x
        pose2d.y = self.y
        pose2d.theta = self.th
        self.pose_pub.publish(pose2d)

        # --- Publish encoder distances ---
        enc_dist = Pose2D()
        enc_dist.x = d_left
        enc_dist.y = d_right
        enc_dist.theta = d
        self.encoder_dist_pub.publish(enc_dist)

        # --- Publish odometry ---
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(self.th / 2)
        quaternion.w = cos(self.th / 2)

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = quaternion
        self.odom_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.twist.twist.linear.x = self.dx
        odom.twist.twist.angular.z = self.dr
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = DiffTf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
