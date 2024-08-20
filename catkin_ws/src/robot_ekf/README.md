# EKF for Pose Estimation

This ROS package implements an Extended Kalman Filter (EKF) for estimating the pose of a robot using odometry and IMU data. It is designed to run on a Raspberry Pi with ROS and provides a robust method for fusing sensor data to improve pose estimation accuracy.

## Overview

The `robot_ekf` package includes:

- **KalmanClass**: Implements the Extended Kalman Filter for state estimation.
- **Caller**: Manages sensor data subscriptions, performs EKF updates, and publishes the estimated pose.

## Features

- **State Prediction**: Predicts robot's state using odometry and IMU data.
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
```roslaunch ekf_pose_estimation ekf.launch```

**Node Description**
ekf_node/kalman_dyn.py: Main script for running the EKF. It subscribes to /odom and /imu_data, performs EKF updates, and publishes the pose to /pose_combined.






