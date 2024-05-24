cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_dwa_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  move_base_msgs
  nav_msgs
  actionlib
  tf
  tf2
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs move_base_msgs nav_msgs actionlib tf tf2 tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/send_goals.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


