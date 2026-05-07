cmake_minimum_required(VERSION 3.0.2)
project(amr_navigation)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

