cmake_minimum_required(VERSION 2.8.3)
project(ros1_vive_msgs)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  Controller.msg
  HeadMountedDisplay.msg
  ViveSystem.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS sensor_msgs geometry_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)