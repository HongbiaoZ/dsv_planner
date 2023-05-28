cmake_minimum_required(VERSION 3.5)
project(volumetric_msgs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LoadMap.srv"
  "srv/SaveMap.srv"
  "srv/SetBoxOccupancy.srv"
  "srv/SetDisplayBounds.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

# ament_auto_package ()
