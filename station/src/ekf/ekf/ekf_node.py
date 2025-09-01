#!/usr/bin/env python3

import math
import numpy as np
from scipy.linalg import block_diag

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


# ----------------------------
# Helper functions
# ----------------------------
def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    y = angle % (2 * np.pi)
    if y > np.pi:
        y -= 2 * np.pi
    return y


def euler_from_quaternion_ros2(quat):
    """Convert a geometry_msgs Quaternion to Euler angles (roll, pitch, yaw)"""
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, +1.0), -1.0)
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


# ----------------------------
# Kalman Class
# ----------------------------
class KalmanClass:
    def __init__(self, x, P, node: Node):
        self.node = node
        self.not_first_time = False
        self.previous_x = np.matrix(x).reshape(-1, 1)
        self.previous_P = np.diag(P)
        self.estimated_x = self.previous_x
        self.estimated_P = self.previous_P

        self.C_redu = np.matrix(
            [
                [1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0],
                [0, 0, 1, 0, 0],
                [0, 0, 0, 1, 0],
                [0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1],
            ]
        )
        self.pub = node.create_publisher(Pose2D, "/pose_combined", 10)

    def predict(self, T, sigma_v, sigma_omega):
        self.predicted_x = np.copy(self.estimated_x)
        self.predicted_x[2, 0] = normalize_angle(
            self.estimated_x[2, 0] + self.estimated_x[4, 0] * T
        )
        self.predicted_x[0, 0] = self.estimated_x[0, 0] + self.estimated_x[
            3, 0
        ] * T * np.cos(self.predicted_x[2, 0])
        self.predicted_x[1, 0] = self.estimated_x[1, 0] + self.estimated_x[
            3, 0
        ] * T * np.sin(self.predicted_x[2, 0])

        ang = normalize_angle(self.estimated_x[2, 0] + T * self.estimated_x[4, 0])
        d_f = np.matrix(
            [
                [T * np.cos(ang), -T * self.estimated_x[3, 0] * np.sin(ang)],
                [T * np.sin(ang), T * self.estimated_x[3, 0] * np.cos(ang)],
                [0, T],
                [1, 0],
                [0, 1],
            ]
        )
        d_f_prime = np.matrix(
            [
                [
                    1,
                    0,
                    -T * self.estimated_x[3, 0] * np.sin(ang),
                    T * np.cos(ang),
                    -T * self.estimated_x[3, 0] * np.sin(ang),
                ],
                [
                    0,
                    1,
                    T * self.estimated_x[3, 0] * np.cos(ang),
                    T * np.sin(ang),
                    T * self.estimated_x[3, 0] * np.cos(ang),
                ],
                [0, 0, 1, 0, T],
                [0, 0, 0, 1, 0],
                [0, 0, 0, 0, 1],
            ]
        )

        var_Q = np.matrix([[sigma_v**2, 0], [0, sigma_omega**2]])
        Q = d_f @ var_Q @ d_f.T
        self.predicted_P = d_f_prime @ self.estimated_P @ d_f_prime.T + Q

        self.previous_x = self.predicted_x
        self.previous_P = self.predicted_P

    def estimate(self, measure):
        if not measure.I_see_something:
            return None

        z = np.matrix(
            [
                [measure.odom_x],
                [measure.odom_y],
                [measure.odom_theta],
                [measure.odom_v],
                [measure.imu_theta],
                [measure.imu_omega],
            ]
        )
        C = self.C_redu
        R = np.matrix(block_diag(measure.odom_covariance, measure.imu_covariance))

        S = C @ self.previous_P @ C.T + R
        S += np.eye(S.shape[0]) * 1e-6

        try:
            K = self.previous_P @ C.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError as e:
            self.node.get_logger().warn(f"Matrix inversion failed: {e}")
            return None

        error = z - (C @ self.previous_x)
        error[2, 0] = normalize_angle(error[2, 0])

        if self.not_first_time:
            self.estimated_x = self.previous_x + K @ error
            self.estimated_x[2, 0] = normalize_angle(self.estimated_x[2, 0])
            mat = np.eye(5) - K @ C
            self.estimated_P = mat @ self.previous_P
        else:
            self.not_first_time = True

        self.node.get_logger().info(
            f"Estimated Pose: x={self.estimated_x[0, 0]:.3f}, y={self.estimated_x[1, 0]:.3f}, theta={self.estimated_x[2, 0]:.3f}"
        )
        return error

    def publish_message(self):
        msg_pose = Pose2D()
        msg_pose.x = float(self.estimated_x[0, 0])
        msg_pose.y = float(self.estimated_x[1, 0])
        msg_pose.theta = float(self.estimated_x[2, 0]) * (180.0 / math.pi)
        self.pub.publish(msg_pose)


# ----------------------------
# EKF Node
# ----------------------------
class EKFNode(Node):
    def __init__(self):
        super().__init__("ekf_dyn")

        # Parameters
        self.declare_parameter("odom_covariance", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("imu_covariance", [0.0, 0.0])

        self.odom_covariance = np.diag(self.get_parameter("odom_covariance").value)
        self.imu_covariance = np.diag(self.get_parameter("imu_covariance").value)

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.odom_v = 0.0
        self.imu_theta = 0.0
        self.imu_omega = 0.0
        self.I_see_something = False

        self.create_subscription(Odometry, "/odom", self.callback_odom, 10)
        self.create_subscription(Imu, "/imu/data", self.callback_imu, 10)

        self.ekf = KalmanClass([0, 0, 0, 0, 0], [1, 1, 1, 1, 1], self)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

    def callback_odom(self, msg):
        _, _, yaw = euler_from_quaternion_ros2(msg.pose.pose.orientation)
        self.odom_theta = yaw
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.I_see_something = True

    def callback_imu(self, msg):
        _, _, yaw = euler_from_quaternion_ros2(msg.orientation)
        self.imu_theta = yaw
        self.imu_omega = msg.angular_velocity.z
        self.I_see_something = True

    def timer_callback(self):
        self.odom_covariance = np.diag(self.get_parameter("odom_covariance").value)
        self.imu_covariance = np.diag(self.get_parameter("imu_covariance").value)
        self.ekf.predict(0.1, 0.1, 0.1)
        self.ekf.estimate(self)
        self.ekf.publish_message()


# ----------------------------
# Main
# ----------------------------
def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
