cmake_minimum_required(VERSION 2.8.3)
project(dsvp_launch)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  tf
  geometry_msgs
  pcl_conversions
  pcl_ros
  sensor_msgs
  nav_msgs
  dsvplanner
  graph_planner
)

include_directories(${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
  LIBRARIES ${catkin_LIBRARIES}
  CATKIN_DEPENDS tf dsvplanner graph_planner
)

add_executable(exploration src/exploration.cpp)
target_link_libraries(exploration
  ${catkin_LIBRARIES}
)
add_dependencies(exploration ${catkin_EXPORTED_TARGETS})

add_executable(navigationBoundary src/navigationBoundary.cpp)
target_link_libraries(navigationBoundary ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(navigationBoundary ${catkin_EXPORTED_TARGETS})
