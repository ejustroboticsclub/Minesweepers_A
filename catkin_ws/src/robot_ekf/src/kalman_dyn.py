#!/usr/bin/env python3

import numpy as np
import rospy
from scipy.linalg import block_diag
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

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

    def publish_message(self, caller_obj):
        pub = rospy.Publisher('/pose_combined', Pose2D, queue_size=10)
        msg_pose = Pose2D()
        x = self.estimated_x[0, 0]
        y = self.estimated_x[1, 0]
        theta = self.estimated_x[2, 0]
        msg_pose.x = x
        msg_pose.y = y
        msg_pose.theta = theta * (180/3.14)
        pub.publish(msg_pose)

class Caller:
    def __init__(self):
        rospy.init_node('ekf_dyn')

        # Initialize covariance values from parameters
        self.odom_covariance = np.diag(rospy.get_param('~odom_covariance', [0.0, 0.0, 0.0, 0.0]))
        self.imu_covariance = np.diag(rospy.get_param('~imu_covariance', [0.0, 0.0]))

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.odom_v = 0.0
        self.imu_theta = 0.0
        self.imu_omega = 0.0
        self.I_see_something = False
        self.time_stamp = None

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odom)
        self.imu_sub = rospy.Subscriber('/imu_data', Imu, self.callback_imu)

    def callback_odom(self, msg):
        self.time_stamp = msg.header.stamp
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_theta = euler_from_quaternion([msg.pose.pose.orientation.x,
                                                msg.pose.pose.orientation.y,
                                                msg.pose.pose.orientation.z,
                                                msg.pose.pose.orientation.w])[2]
        self.odom_v = np.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
        self.I_see_something = True

    def callback_imu(self, msg):
        self.imu_theta = euler_from_quaternion([msg.orientation.x,
                                                msg.orientation.y,
                                                msg.orientation.z,
                                                msg.orientation.w])[2]
        self.imu_omega = msg.angular_velocity.z
        self.I_see_something = True

    def run(self):
        x0 = [0, 0, 0, 0, 0]  # Initial state
        P0 = [1, 1, 1, 1, 1]  # Initial covariance
        ekf = KalmanClass(x0, P0)
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            # Update covariance values from ROS parameters
            self.odom_covariance = np.diag(rospy.get_param('~odom_covariance', [0.0, 0.0, 0.0, 0.0]))
            self.imu_covariance = np.diag(rospy.get_param('~imu_covariance', [0.0, 0.0]))

            ekf.predict(0.1, 0.1, 0.1)  # Time step, sigma_v(Velocity Noise), sigma_omega (Angular Velocity)
            ekf.estimate(self)
            ekf.publish_message(self)
            rate.sleep()

if __name__ == '__main__':
    try:
        caller = Caller()
        caller.run()
    except rospy.ROSInterruptException:
        pass
