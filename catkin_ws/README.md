### Dependencies
- `inputs`
- `opencv-python`
- `socket`
- `math`
- `pygame`
- `scipy`
- `numpy`
### Arduino Code

#### Includes:
- Metal detector
- Alarm (buzzer)
- Magnet (gripper)
- Encoders

#### Libraries:
- `QuadratureEncoder`
- `ros`
- `rosserial`
- `rosserial_arduino`
- `EnableInterrupt`
- `std_msgs/Int32.h`
- `std_msgs/Int16.h`
- `std_msgs/Int8.h`

#### Packages:
- `joy`
- `diffodom`
- `robot_ekf`

#### Message Types:

  - **joy package:**
    - `geometry_msgs`
    - `std_msgs`

  - **diffodom package:**
    - `sensor_msgs`
    - `geometry_msgs`
    - `nav_msgs`
    - `std_msgs`
    - `tf`

  - **robot_ekf package:**
    - `geometry_msgs`
    - `tf`
    - `sensor_msgs`
    - `nav_msgs`
