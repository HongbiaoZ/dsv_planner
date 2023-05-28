cmake_minimum_required(VERSION 3.5)
project(volumetric_map_base)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies ()

include_directories(
  include
  ${ament_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  /lib/x86_64-linux-gnu
)

#############
# LIBRARIES #
#############
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/world_base.cc
)
target_link_libraries(${PROJECT_NAME} glog)

##########
# EXPORT #
##########
ament_auto_package ()
