cmake_minimum_required(VERSION 3.0.2)
project(amr_teensy_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  std_msgs
)

catkin_package()

catkin_install_python(
  PROGRAMS
    src/teensy_bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

