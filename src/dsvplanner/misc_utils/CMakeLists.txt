cmake_minimum_required(VERSION 3.5)
project(misc_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(PCL REQUIRED COMPONENTS common io)

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


file(GLOB_RECURSE misc_utils_SOURCES src/*.cpp src/*.c)
add_library(${PROJECT_NAME} SHARED ${misc_utils_SOURCES})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} 
    rclcpp
    geometry_msgs
    nav_msgs
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
        rclcpp
        geometry_msgs
        nav_msgs
)

install(
    DIRECTORY include/
    DESTINATION include
)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} # 告诉ros2有这么个目标（可执行文件或者库）
        EXPORT ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

 ament_package()