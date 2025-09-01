cmake_minimum_required(VERSION 3.10)
project(joy)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python modules
install(
  DIRECTORY src/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Install executable scripts
install(PROGRAMS
  src/control_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
