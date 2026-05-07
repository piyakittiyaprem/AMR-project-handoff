cmake_minimum_required(VERSION 3.0.2)
project(amr_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  urdf
  xacro
  robot_state_publisher
)

catkin_package()

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

