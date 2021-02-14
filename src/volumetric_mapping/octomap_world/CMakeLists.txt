#cmake_minimum_required(VERSION 2.8.3)
#project(octomap_world)

#set(CMAKE_BUILD_TYPE Release)
#find_package(catkin REQUIRED)

#add_definitions(-std=c++11)

##############
## LIBRARIES #
##############
#add_library(${PROJECT_NAME} src/octomap_world.cc src/octomap_manager.cc)
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

###############################
### Declare a C++ executable ##
###############################
#add_executable(octomap_manager src/octomap_manager_node.cc)
#add_dependencies(octomap_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(octomap_manager ${PROJECT_NAME})

cmake_minimum_required(VERSION 2.8.3)
project(octomap_world)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/octomap_world.cc
  src/octomap_manager.cc
)
target_link_libraries(${PROJECT_NAME} glog)
############
# BINARIES #
############
cs_add_executable(octomap_manager
  src/octomap_manager_node.cc
)
target_link_libraries(octomap_manager ${PROJECT_NAME} glog)

##########
# EXPORT #
##########
cs_install()
cs_export()
