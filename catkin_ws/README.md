### How to launch all laptop nodes
  First, run the stream to make yolonode.py be able to send the readings to *detection_status* file
  ``` 
  python ~/catkin_ws/src/yolo/src/yolo.py
  ```
  Then سمـّي
  ```
  python3 sammy.py
  ```
sammy → سمـّي
### Building the packages
  ```
  catkin_make
  ```
  - some times it stucks when using catkin_make so another solution is specifying number of jobs will be made at the same time, I prefer to make it only **1** job which is so good in Raspberry Pi by using :
    ```
    catkin_make -j1
    ```
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
