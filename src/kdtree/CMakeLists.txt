cmake_minimum_required(VERSION 2.8.3)
project(kdtree)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME} src/kdtree.c)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
