cmake_minimum_required(VERSION 2.8.3)
project(volumetric_msgs)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    std_msgs
    message_generation
)
find_package(cmake_modules REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  LoadMap.srv
  SaveMap.srv
  SetBoxOccupancy.srv
  SetDisplayBounds.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs std_msgs
)

