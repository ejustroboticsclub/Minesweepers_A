# Differential Odometry:
This ROS node, named `diff_tf`, is responsible for calculating and broadcasting the odometry of a differential drive robot using wheel encoder data and IMU (Inertial Measurement Unit) readings.
## DiffTf Node

The `diff_tf` node calculates and publishes odometry information for a differential drive robot based on wheel encoder ticks and IMU data. The node is essential for tracking the robot's position and orientation in the environment. Here's a summary of what this node does:

### Key Features:

1. **Initialization and Parameters**:
   - The node initializes various parameters such as the number of encoder ticks per meter for each wheel, the base width of the robot, and frame IDs for the robot's base and odometry.
   - It also handles the encoder value wrapping to manage cases where the encoder ticks exceed their maximum or minimum values.

2. **Subscribers**:
   - The node subscribes to the `left_ticks` and `right_ticks` topics, which provide the encoder ticks from the left and right wheels, respectively.
   - It also subscribes to the `/imu_data` topic, which provides orientation data from the IMU.

3. **Odometry Calculation**:
   - The node computes the distance traveled by each wheel using the encoder ticks and the number of ticks per meter.
   - It calculates the robot's position (x, y) and orientation (theta) based on the movement of the wheels and the yaw (orientation) provided by the IMU.
   - The velocities in the x-direction and rotation are computed and published as part of the odometry data.

4. **Publishing**:
   - The node publishes the odometry information to the `odom` topic using the `nav_msgs/Odometry` message type.
   - It also publishes the 2D pose of the robot to the `/robot_2d` topic using the `geometry_msgs/Pose2D` message type.
   - Additionally, the distance traveled by each wheel is published to the `/encoder_dist` topic.
   - A transform is broadcasted between the odometry frame and the base frame using the `tf` library, allowing other ROS nodes to know the robot's position in the environment.

5. **Main Loop**:
   - The `spin` function is the main loop of the node, which continually updates and publishes the odometry data at a specified rate.

### Usage:
To run the `diff_tf` node, simply execute the following command in your terminal:

```bash
rosrun diffodom diffodom.py
```

Ensure that the topics for `left_ticks`, `right_ticks`, and `imu_data` are correctly published by your robot's hardware or simulation environment.

#### Message Types:

 `sensor_msgs`
 `geometry_msgs`
 `nav_msgs`
 `std_msgs`
 `tf`
