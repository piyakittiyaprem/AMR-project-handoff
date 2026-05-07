cmake_minimum_required(VERSION 3.0.2)
project(amr_autonomous_mapping)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  tf2_ros
)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/slam_pose_to_odom.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
