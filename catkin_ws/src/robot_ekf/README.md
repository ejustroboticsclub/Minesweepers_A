# EKF for Pose Estimation

This ROS package implements an Extended Kalman Filter (EKF) to estimate the pose of a robot using odometry and IMU data. It is designed to fuse sensor data to improve pose estimation accuracy.

## Overview

The `robot_ekf` package includes:

- **KalmanClass**: Implements the Extended Kalman Filter for state estimation.
- **Caller**: Manages sensor data subscriptions, performs EKF updates, and publishes the estimated pose.

## Features

- **State Prediction**: Predicts the robot's state using odometry and IMU data.
- **State Update**: Updates the state estimate with incoming sensor measurements.
- **Message Publishing**: Publishes the estimated pose to the `/pose_combined` topic.

## Installation

1. **Install Dependencies**

   Install the required Python packages:

   ```
   pip3 install numpy scipy rospy
2.**Clone the Repository**

Clone this repository to your ROS workspace:

```
cd ~/catkin_ws/src
```

```
git clone https://github.com/ahmedanwar123/robot_ekf.git
```
3.**Build the Package**
Build the package in your catkin workspace:
```
cd ~/catkin_ws
```
```
catkin_make
```
4.**Source the Workspace**
Source the setup file to make the package available:
```source devel/setup.bash```

## Configuration
Set the following parameters for EKF in your ROS launch file or using the rosparam command:

~odom_covariance: Covariance matrix for odometry measurements.
~imu_covariance: Covariance matrix for IMU measurements.

**Example Configuration**
```
<param name="odom_covariance" value="[0.1, 0.1, 0.1, 0.1]" />
<param name="imu_covariance" value="[0.1, 0.1]" />
```
**Running the node**
```
roslaunch robot_ekf kalman_dyn.py
```
**Running GUI for tunning covariance**
```
python3 gui.py
```

**Node Description**
ekf_node/kalman_dyn.py: Main script for running the EKF. It subscribes to /odom and /imu_data, performs EKF updates, and publishes the pose to /pose_combined.
note that *dyn* refers to dynamic as covariances can be tuned while working which helped a lot.

To create the message types required in your ROS node, you'll need to create a `.msg` file to define the custom message and update the `CMakeLists.txt` and `package.xml` accordingly. However, since you are only using existing ROS messages like `Pose2D`, `Imu`, and `Odometry`, there's no need to define new messages.

Here's a summary of the message types used in the script:

### Message Types

```markdown
# Pose2D (from geometry_msgs)
# This message represents a 2D pose of a robot, including x, y, and theta (orientation).

float64 x       # X position in meters
float64 y       # Y position in meters
float64 theta   # Orientation in radians

---

# Imu (from sensor_msgs)
# This message contains the measured orientation, angular velocity, and linear acceleration.

Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance

---

# Odometry (from nav_msgs)
# This message contains an estimate of a position and velocity in free space.

Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```

### Note

- **Pose2D:** Used for publishing the combined pose estimated by the Kalman filter.
- **Imu:** Subscribed to get IMU data (orientation and angular velocity).
- **Odometry:** Subscribed to get odometry data (position, orientation, and velocity).




