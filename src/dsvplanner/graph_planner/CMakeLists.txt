cmake_minimum_required(VERSION 3.5)
project(graph_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(misc_utils REQUIRED)
find_package(graph_utils REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GraphPlannerCommand.msg"
  "msg/GraphPlannerStatus.msg"
  DEPENDENCIES geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${ament_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_exe src/graph_planner.cpp src/graph_planner_node.cpp)
target_link_libraries(${PROJECT_NAME}_exe ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_exe
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    pcl_conversions
    misc_utils
    graph_utils
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(${PROJECT_NAME}_exe "${cpp_typesupport_target}")

install(
  DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS ${PROJECT_NAME}_exe 
  DESTINATION lib/${PROJECT_NAME})

ament_package()