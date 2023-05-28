cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0074 NEW)
project(dsvp_launch)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dsvplanner REQUIRED)
find_package(graph_planner REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS} ${ament_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(exploration src/exploration.cpp)
ament_target_dependencies(exploration
    rclcpp
    graph_planner
    dsvplanner
    geometry_msgs
    nav_msgs
    std_msgs
    tf2
)

add_executable(navigationBoundary src/navigationBoundary.cpp)
target_link_libraries(navigationBoundary ${PCL_LIBRARIES})
ament_target_dependencies(navigationBoundary
    rclcpp
    pcl_conversions
    pcl_ros
    sensor_msgs
    nav_msgs
)

install(TARGETS exploration navigationBoundary
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
