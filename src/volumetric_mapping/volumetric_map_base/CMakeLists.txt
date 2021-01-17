cmake_minimum_required(VERSION 2.8.3)
project(volumetric_map_base)

add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/world_base.cc
)

##########
# EXPORT #
##########
cs_install()
cs_export()
