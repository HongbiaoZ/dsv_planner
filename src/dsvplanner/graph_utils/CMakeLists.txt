cmake_minimum_required(VERSION 3.5)
project(graph_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(misc_utils REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate added messages and services with any dependencies listed here
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Edge.msg"
  "msg/Vertex.msg"
  "msg/TopologicalGraph.msg"
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

###########################
## Declare a C++ library ##
###########################

# graph_utils -- provide functions for graph handle
add_library(${PROJECT_NAME}_lib src/graph_utils.cpp)
target_link_libraries(${PROJECT_NAME}_lib ${PCL_LIBRARIES})
target_include_directories(${PROJECT_NAME}_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
) 
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  misc_utils
  geometry_msgs
)
ament_export_targets(${PROJECT_NAME}_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  misc_utils
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(${PROJECT_NAME}_lib "${cpp_typesupport_target}")

##############################
## Declare a C++ executable ##
##############################

# graph_visulization -- generates rviz markers based on the output of graph
add_executable(graph_visualization src/graph_visualization.cpp src/graph_visualization_node.cpp)
ament_target_dependencies(graph_visualization
    rclcpp
    std_msgs
    geometry_msgs
    visualization_msgs
    misc_utils
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(graph_visualization "${cpp_typesupport_target}")

install(
    DIRECTORY include/
    DESTINATION include
)

install(
  DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS ${PROJECT_NAME}_lib # 告诉ros2有这么个目标（可执行文件或者库）
        EXPORT ${PROJECT_NAME}_lib
        DESTINATION lib/${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(TARGETS graph_visualization
  DESTINATION lib/${PROJECT_NAME})

ament_package()



