cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0074 NEW)
project(octomap_world)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(minkindr_conversions REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(volumetric_map_base REQUIRED)
find_package(volumetric_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME} SHARED
  src/octomap_world.cc
  src/octomap_manager.cc
)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_srvs
  minkindr_conversions
  octomap
  octomap_msgs
  octomap_ros
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  sensor_msgs
  volumetric_map_base
  volumetric_msgs
  tf2_geometry_msgs
  visualization_msgs
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_srvs
  minkindr_conversions
  octomap
  octomap_msgs
  octomap_ros
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  sensor_msgs
  volumetric_map_base
  volumetric_msgs
  tf2_geometry_msgs
  visualization_msgs
)

##########
# EXPORT #
##########
install(
    DIRECTORY include/
    DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

ament_package()