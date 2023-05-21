cmake_minimum_required(VERSION 3.5)
project(dsvp_launch)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath,'$ORIGIN/'" )
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-rpath,'$ORIGIN/'" )
set(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}")
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}")
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(dsvplanner REQUIRED)
find_package(graph_planner REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS} ${ament_INCLUDE_DIRS})

add_executable(exploration src/exploration.cpp)
target_link_libraries(exploration ${catkin_LIBRARIES})
ament_target_dependencies(exploration
    rclcpp
    graph_planner
    dsvplanner
    geometry_msgs
    sensor_msgs
)
add_dependencies(exploration ${catkin_EXPORTED_TARGETS})

add_executable(navigationBoundary src/navigationBoundary.cpp)
target_link_libraries(navigationBoundary ${catkin_LIBRARIES} ${PCL_LIBRARIES})
ament_target_dependencies(navigationBoundary
    rclcpp
    pcl_conversions
    pcl_ros
    sensor_msgs
    nav_msgs
)

install(TARGETS exploration navigationBoundary
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY images
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.png" PATTERN "*.svg")

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
