cmake_minimum_required(VERSION 2.8.3)
project(graph_utils)

add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  nav_msgs
  misc_utils
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   TopologicalGraph.msg
   Vertex.msg
   Edge.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES graph_utils
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime nav_msgs misc_utils
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

###########################
## Declare a C++ library ##
###########################

# graph_utils -- provide functions for graph handle
add_library(graph_utils src/graph_utils.cpp)
add_dependencies(graph_utils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(graph_utils ${catkin_LIBRARIES})

##############################
## Declare a C++ executable ##
##############################

# graph_visulization -- generates rviz markers based on the output of graph
add_executable(graph_visualization src/graph_visualization.cpp src/graph_visualization_node.cpp)
add_dependencies(graph_visualization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(graph_visualization ${catkin_LIBRARIES})




