cmake_minimum_required(VERSION 2.8.3)
project(dsvp_launch)

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

add_executable(pc_rename src/pc_rename.cpp)
target_link_libraries(pc_rename
  ${catkin_LIBRARIES}
)
add_dependencies(pc_rename ${catkin_EXPORTED_TARGETS})

add_executable(data_collect_gbp src/data_collect_gbp.cpp)
target_link_libraries(data_collect_gbp
  ${catkin_LIBRARIES}
)
add_dependencies(data_collect_gbp ${catkin_EXPORTED_TARGETS})

add_executable(data_collect_nbvp src/data_collect_nbvp.cpp)
target_link_libraries(data_collect_nbvp
  ${catkin_LIBRARIES}
)
add_dependencies(data_collect_nbvp ${catkin_EXPORTED_TARGETS})

add_executable(data_collect_keypose src/data_collect_keypose.cpp)
target_link_libraries(data_collect_keypose
  ${catkin_LIBRARIES}
)
add_dependencies(data_collect_keypose ${catkin_EXPORTED_TARGETS})

add_executable(data_process_mbp src/data_process_mbp.cpp)
target_link_libraries(data_process_mbp
  ${catkin_LIBRARIES}
)
add_dependencies(data_process_mbp ${catkin_EXPORTED_TARGETS})

add_executable(motion src/motion.cpp)
target_link_libraries(motion
  ${catkin_LIBRARIES}
)
add_dependencies(motion ${catkin_EXPORTED_TARGETS})
