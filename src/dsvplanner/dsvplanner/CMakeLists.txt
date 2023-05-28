cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0074 NEW)
project(dsvplanner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(misc_utils REQUIRED)
find_package(graph_utils REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(octomap_world REQUIRED)
find_package(kdtree REQUIRED)
find_package(tf2 REQUIRED)
find_package(message_filters REQUIRED)
find_package(graph_planner REQUIRED)
find_package(PCL REQUIRED)
find_package(rosidl_default_generators REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Dsvplanner.srv"
  "srv/CleanFrontier.srv"
  DEPENDENCIES std_msgs geometry_msgs
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

## Declare a C++ library
add_library(dsvPlannerLib SHARED src/drrtp.cpp src/drrt.cpp src/dual_state_graph.cpp src/dual_state_frontier.cpp src/grid.cpp)
target_link_libraries(dsvPlannerLib ${PCL_LIBRARIES})
ament_target_dependencies(dsvPlannerLib
    rclcpp
    geometry_msgs
    sensor_msgs
    nav_msgs
    std_msgs
    visualization_msgs
    octomap_world
    misc_utils
    graph_utils
    kdtree
    tf2
    message_filters
    graph_planner
)
rosidl_target_interfaces(dsvPlannerLib ${PROJECT_NAME} "rosidl_typesupport_cpp")

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_exe src/drrtp_node.cpp)
target_link_libraries(${PROJECT_NAME}_exe  ${PCL_LIBRARIES} dsvPlannerLib)
ament_target_dependencies(${PROJECT_NAME}_exe 
    rclcpp
)
rosidl_target_interfaces(${PROJECT_NAME}_exe ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS dsvPlannerLib ${PROJECT_NAME}_exe 
  EXPORT dsvPlannerLib
  DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

ament_package()