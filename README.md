# Minesweepers_A 2024
**Minesweepers Competition'2024 Team A**

## Team Members:

### Team Leaders:
- Mariam Gadallah
- Ahmed Anwar

### Software Team Members:
- Noura Maklad
- Mohamed Mahdy

### Electrical Team Members:
- Youssef Hatem
- Mahmoud Sayed

### Mechanical Team Members:
- Mahmoud Essam
- Alaaeldin Manaa
- Omar Elgamal

## Workspaces:
- catkin_ws (Laptop Workspace)
- rpi_ws (Raspberry Pi Workspace)


## To-Do List:

- Make the robot move         (LEFT Stick or Buttons on the LEFT side)
  - [x] In Progress 
  - [x] Done
- Assign buttons for gripper  (RIGHT Stick or Buttons on the RIGHT side)
  - [x] In Progress 
  - [X] Done
- Mapping
  - IMU Calibration code (Madgwick Filter)
    - [x] In Progress 
    - [x] Done
  - Kalman Filter
    - [x] In Progress 
    - [x] Done
- Mapping GUI 
  - [x] In Progress 
  - [x] Done
- Streaming
  - [x] In Progress 
  - [x] Done
- Launch Files preparation 
  - [x] In Progress 
  - [x] Done
- Assemble electrical components 
  - [X] In Progress 
  - [ ] Done
- Fabricate mechanical parts 
  - [x] In Progress 
  - [ ] Done
- Assemble mechanical parts 
  - [x] In Progress 
  - [ ] Done
- Test and integrate all components 
  - [x] In Progress 
  - [ ] Done
- Prepare Report and Video
  - [x] In Progress 
  - [ ] Done
- Prepare for the final presentation 
  - [ ] In Progress 
  - [ ] Done
--------------------------------------------------------------------
## How to launch

<div align="center">

**سمـّي و دوّر**

</div>

- Start by launching `sammy.py` on your PC using `python3 sammy.py`.
- Then, on RPi, launch `dawar.py` by using `python3 dawar.py`.

--------------------------------------------------------------------

# Notes:
**dawar.py is for launching the laptop nodes**
  - For Arduino ROS lib navigate to the following link:
    - [Arduino ROS](https://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)
  - Must add ip for pc in pi host list (/etc/hosts)  in the following format : ip username ,and vice versa
  - For MPU6050 Package (it works for MPU 9250 Also) you have to install this on your RPi:
```
sudo apt-get install ros-${ROS_DISTRO}-turtle-tf2 ros-${ROS_DISTRO}-tf2-tools ros-${ROS_DISTRO}-tf
```
## Controller Alignemnt
![Controller](Controller.png)

