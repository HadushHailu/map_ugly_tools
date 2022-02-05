cmake_minimum_required(VERSION 3.0.2)
project(map_ugly_tools)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)
add_message_files(
  FILES
  DeleteRectObstacle.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES map_ugly_tools
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
${catkin_INCLUDE_DIRS}
)
catkin_install_python(PROGRAMS scripts/first_obstacle_cell_etc.py scripts/down_sample_map.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

