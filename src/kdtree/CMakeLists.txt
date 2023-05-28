cmake_minimum_required(VERSION 3.5)
project(kdtree)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)

###########
## Build ##
###########

include_directories(include ${ament_INCLUDE_DIRS})

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME} SHARED src/kdtree.c)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
    DIRECTORY include/
    DESTINATION include
)

install(TARGETS ${PROJECT_NAME} # 告诉ros2有这么个目标（可执行文件或者库）
        EXPORT ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

ament_package()