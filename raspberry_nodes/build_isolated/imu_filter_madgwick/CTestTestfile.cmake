# CMake generated Testfile for 
# Source directory: /home/pi/catkin_ws/src/imu_filter_madgwick
# Build directory: /home/pi/catkin_ws/build_isolated/imu_filter_madgwick
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_imu_filter_madgwick_gtest_imu_filter_madgwick-madgwick_test "/home/pi/catkin_ws/build_isolated/imu_filter_madgwick/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/pi/catkin_ws/build_isolated/imu_filter_madgwick/test_results/imu_filter_madgwick/gtest-imu_filter_madgwick-madgwick_test.xml" "--return-code" "/home/pi/catkin_ws/devel_isolated/imu_filter_madgwick/lib/imu_filter_madgwick/imu_filter_madgwick-madgwick_test --gtest_output=xml:/home/pi/catkin_ws/build_isolated/imu_filter_madgwick/test_results/imu_filter_madgwick/gtest-imu_filter_madgwick-madgwick_test.xml")
set_tests_properties(_ctest_imu_filter_madgwick_gtest_imu_filter_madgwick-madgwick_test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/pi/catkin_ws/src/imu_filter_madgwick/CMakeLists.txt;93;catkin_add_gtest;/home/pi/catkin_ws/src/imu_filter_madgwick/CMakeLists.txt;0;")
subdirs("gtest")
