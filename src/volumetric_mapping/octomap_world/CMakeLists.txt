cmake_minimum_required(VERSION 2.8.3)
project(octomap_world)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


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
