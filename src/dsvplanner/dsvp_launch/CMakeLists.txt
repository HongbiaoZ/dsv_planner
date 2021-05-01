cmake_minimum_required(VERSION 2.8.3)
project(dsvp_launch)

add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  tf
  geometry_msgs
  pcl_conversions
  pcl_ros
  sensor_msgs
  visualization_msgs
  graph_utils
  nav_msgs
  dsvplanner
  graph_planner
)

include_directories(${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
  LIBRARIES ${catkin_LIBRARIES}
  CATKIN_DEPENDS tf visualization_msgs graph_utils dsvplanner graph_planner
)

add_executable(exploration_with_graph_planner src/exploration_with_graph_planner.cpp)
target_link_libraries(exploration_with_graph_planner
  ${catkin_LIBRARIES}
)
add_dependencies(exploration_with_graph_planner ${catkin_EXPORTED_TARGETS})
