# motor_controller ROS Node

## Overview

The Motor Controller ROS Node is a Python-based ROS node designed to control a differential drive robot using PWM signals to drive four motors. It subscribes to the `/cmd_vel` topic to receive velocity commands and translates them into motor speeds and directions.

## Features

- **Subscribes** to `/cmd_vel` for velocity commands.
- **Controls** four DC motors via PWM signals.
- **Calculates** and sets motor speeds based on linear and angular velocities.
- **Logs** motor speeds for debugging and monitoring.

## Dependencies

- **ROS** (Robot Operating System) - Ensure that ROS is installed and properly configured.
- **RPi.GPIO** - Python library for controlling GPIO pins on a Raspberry Pi.

## Installation

1. **Install Dependencies:**
    ```bash
    pip install RPi.GPIO
    ```

2. **Build Your ROS Workspace:**
    - Place this node inside your ROS workspace's `src` directory.
    - Build the workspace:
      ```bash
      cd ~/catkin_ws
      catkin_make
      ```

## Configuration

- **GPIO Pins:**
  - MOTOR1: PWM pin 9, DIR pin 8
  - MOTOR2: PWM pin 10, DIR pin 11
  - MOTOR3: PWM pin 6, DIR pin 7
  - MOTOR4: PWM pin 5, DIR pin 4

- **Robot Parameters:**
  - `wheel_radius`: Radius of the robot's wheels in meters (default: 0.12).
  - `robot_radius`: Distance from the robot's center to the wheels in meters (default: 0.5).
  - `max_speed`: Maximum speed of the robot in meters per second (default: 1.0).
  - `max_motor_speed`: Maximum motor speed (PWM duty cycle) (default: 255).
    
## Message Types

- **`geometry_msgs/Twist`**
  - **Description:** Represents velocity commands for the robot.
  - **Fields:**
    - `linear.x`: Linear velocity in meters per second.
    - `angular.z`: Angular velocity in radians per second.

- **`std_msgs/Int16`** (not used in this node, but included for completeness)
  - **Description:** Represents 16-bit integer messages.
  - **Fields:**
    - `data`: 16-bit integer value.

## Topics

- **`/cmd_vel`** (Subscriber)
  - **Message Type:** `geometry_msgs/Twist`
  - **Description:** Receives velocity commands for the robot. This topic is used to control the robot's linear and angular velocities.

## Usage

1. **Launch the Node:**
    ```bash
    roslaunch motors motor_controller.py
    ```

2. **Send Velocity Commands:**
    - Publish messages to the `/cmd_vel` topic with `geometry_msgs/Twist` type to control the robot.
    ```bash
    rostopic pub /cmd_vel geometry_msgs/Twist -- '[linear_x, 0.0, 0.0]' '[0.0, 0.0, angular_z]'
    ```

## Code Description

- **Initialization (`__init__`):**
  - Initializes the ROS node.
  - Sets up GPIO pins and PWM channels for motor control.
  
- **Callback Function (`cmd_vel_callback`):**
  - Updates `linear_vel` and `angular_vel` based on incoming `/cmd_vel` messages.

- **Motor Speed Calculation (`publish_motor_speeds`):**
  - Converts linear and angular velocities to motor speeds.
  - Sets the speed and direction of each motor using PWM signals.

- **Main Loop (`run`):**
  - Continuously publishes motor speeds at a rate of 10 Hz.

## Cleanup

- **GPIO Cleanup:**
  - Ensures that GPIO pins are cleaned up upon node shutdown to avoid conflicts.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Raspberry Pi Foundation for GPIO control library.
- ROS community for the Robot Operating System framework.

