#!/usr/bin/env python3

import numpy as np
import rospy
from scipy.linalg import block_diag
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Quaternion


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    y = angle % (2 * np.pi)    # force in range [0, 2*pi)
    if y > np.pi:             # move to [-pi, pi)
        y -= 2 * np.pi
    return y


class KalmanClass:
    def __init__(self, x, P):
        self.not_first_time = False
        self.previous_x = np.matrix(x).reshape(-1, 1)  # Ensure correct shape (5, 1)
        self.previous_P = np.diag(P)
        self.estimated_x = self.previous_x
        self.estimated_P = self.previous_P

        self.C_redu = np.matrix([[1, 0, 0, 0, 0],
                                 [0, 1, 0, 0, 0],
                                 [0, 0, 1, 0, 0],
                                 [0, 0, 0, 1, 0],
                                 [0, 0, 1, 0, 0],
                                 [0, 0, 0, 0, 1]])

    def predict(self, T, sigma_v, sigma_omega):
        # State prediction
        self.predicted_x = np.copy(self.estimated_x)
        self.predicted_x[2, 0] = normalize_angle(self.estimated_x[2, 0] + self.estimated_x[4, 0] * T)
        self.predicted_x[0, 0] = self.estimated_x[0, 0] + self.estimated_x[3, 0] * T * np.cos(self.predicted_x[2, 0])
        self.predicted_x[1, 0] = self.estimated_x[1, 0] + self.estimated_x[3, 0] * T * np.sin(self.predicted_x[2, 0])
        
        # Jacobian matrix
        ang = normalize_angle(self.estimated_x[2, 0] + T * self.estimated_x[4, 0])
        d_f = np.matrix([[T * np.cos(ang), -T * self.estimated_x[3, 0] * np.sin(ang)],
                         [T * np.sin(ang), T * self.estimated_x[3, 0] * np.cos(ang)],
                         [0, T],
                         [1, 0],
                         [0, 1]])
        d_f_prime = np.matrix([[1, 0, -T * self.estimated_x[3, 0] * np.sin(ang), T * np.cos(ang), -T * self.estimated_x[3, 0] * np.sin(ang)],
                               [0, 1, T * self.estimated_x[3, 0] * np.cos(ang), T * np.sin(ang), T * self.estimated_x[3, 0] * np.cos(ang)],
                               [0, 0, 1, 0, T],
                               [0, 0, 0, 1, 0],
                               [0, 0, 0, 0, 1]])
        
        var_Q = np.matrix([[sigma_v**2, 0], [0, sigma_omega**2]])
        Q = d_f @ var_Q @ d_f.T
        self.predicted_P = d_f_prime @ self.estimated_P @ d_f_prime.T + Q

        # Update state and covariance
        self.previous_x = self.predicted_x
        self.previous_P = self.predicted_P

    def estimate(self, measure):
        if measure.I_see_something:
            z = np.matrix([[measure.odom_x], [measure.odom_y],
                           [measure.odom_theta], [measure.odom_v],
                           [measure.imu_theta], [measure.imu_omega]])
            C = self.C_redu
            R = np.matrix(block_diag(measure.odom_covariance, measure.imu_covariance))
        else:
            return None

        S = C @ self.previous_P @ C.T + R
        # Regularize S to avoid singular matrix
        S += np.eye(S.shape[0]) * 1e-6

        try:
            self.K = self.previous_P @ C.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError as e:
            rospy.logwarn(f"Matrix inversion failed: {e}")
            return None

        error = z - (C @ self.previous_x)
        error[2, 0] = normalize_angle(error[2, 0])

        if self.not_first_time:
            self.estimated_x = self.previous_x + self.K @ error
            self.estimated_x[2, 0] = normalize_angle(self.estimated_x[2, 0])
            mat = np.eye(5) - self.K @ C
            self.estimated_P = mat @ self.previous_P
        else:
            self.not_first_time = True

        return error

    def callback_velocity(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z
        v = np.sqrt(vx**2 + vy**2)
        self.estimated_x[3, 0] = v
        self.estimated_x[4, 0] = omega

    def update_velocity(self):
        rospy.Subscriber('/mobile_base/commands/velocity', Twist, self.callback_velocity)

    def publish_message(self, caller_obj):
        pub = rospy.Publisher('/odom_combined', Odometry, queue_size=10)
        msg_odom = Odometry()
        x = self.estimated_x[0, 0]
        y = self.estimated_x[1, 0]
        z = 0
        quatern = quaternion_from_euler(0, 0, self.estimated_x[2, 0])
        msg_odom.header.stamp = caller_obj.time_stamp
        msg_odom.header.frame_id = 'base_footprint'
        msg_odom.pose.pose.position = Point(x, y, z)
        msg_odom.pose.pose.orientation = Quaternion(*quatern)
        p = np.diag([self.estimated_P[0, 0], self.estimated_P[1, 1], 10000000, 10000000, 1000000])
        msg_odom.pose.covariance = tuple(p.ravel().tolist())
        pub.publish(msg_odom)


class Caller:
    def __init__(self):
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.odom_v = 0.0
        self.imu_theta = 0.0
        self.imu_omega = 0.0
        self.odom_covariance = np.diag([0.1, 0.1, 0.1, 0.1])  # Example covariance values
        self.imu_covariance = np.diag([0.01, 0.01])  # Example covariance values
        self.I_see_something = False
        self.time_stamp = None

    def read_sensors(self):
        self.call_odom()
        self.call_imu()

    def call_odom(self):
        try:
            msg = rospy.wait_for_message('/odom', Odometry, timeout=0.1)
            self.time_stamp = msg.header.stamp
            self.odom_x = msg.pose.pose.position.x
            self.odom_y = msg.pose.pose.position.y
            self.odom_theta = euler_from_quaternion([msg.pose.pose.orientation.x,
                                                    msg.pose.pose.orientation.y,
                                                    msg.pose.pose.orientation.z,
                                                    msg.pose.pose.orientation.w])[2]
            self.odom_v = np.sqrt(msg.twist.twist.linear.x**2 +
                                  msg.twist.twist.linear.y**2)
            self.I_see_something = True
        except Exception as e:
            rospy.logwarn(f"No odom data: {e}")
            self.I_see_something = False

    def call_imu(self):
        try:
            msg = rospy.wait_for_message('/imu_data', Imu, timeout=0.1)
            self.imu_theta = euler_from_quaternion([msg.orientation.x,
                                                   msg.orientation.y,
                                                   msg.orientation.z,
                                                   msg.orientation.w])[2]
            self.imu_omega = msg.angular_velocity.z
            self.imu_covariance = np.diag([msg.linear_acceleration_covariance[0],
                                           msg.angular_velocity_covariance[2]])
            self.I_see_something = True
        except Exception as e:
            rospy.logwarn(f"No IMU data: {e}")
            self.I_see_something = False

    def run(self):
        rospy.init_node('ekf_node')
        x0 = [0, 0, 0, 0, 0]  # Initial state
        P0 = [1, 1, 1, 1, 1]  # Initial covariance
        ekf = KalmanClass(x0, P0)
        ekf.update_velocity()

        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.read_sensors()
            ekf.predict(0.1, 0.1, 0.1)  # Example T, sigma_v, sigma_omega
            ekf.estimate(self)
            ekf.publish_message(self)
            rate.sleep()


if __name__ == "__main__":
    try:
        caller = Caller()
        caller.run()
    except rospy.ROSInterruptException:
        pass
