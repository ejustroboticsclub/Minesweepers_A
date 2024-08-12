# motors node
- on RPi
  - rosrun motors motor_controller.py
# Notes:
- Decompress orocos_kinematics_dynamics and start building your packages.
- If the packages don't build use catkin_make_isolated instead of (catkin_make)
- If it lags you have to build it with one job at a time : catkin_make_isolated -j1
