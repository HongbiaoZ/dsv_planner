cmake_minimum_required(VERSION 3.5)
project(minkindr_conversions)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(minkindr REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(builtin_interfaces REQUIRED)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
  /usr/lib/x86_64-linux-gnu
  /lib/x86_64-linux-gnu
)

add_library(${PROJECT_NAME} SHARED
  src/kindr_msg.cc
  src/kindr_tf.cc)
target_link_libraries(${PROJECT_NAME} glog)
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  minkindr
  tf2
  tf2_eigen
  builtin_interfaces
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    minkindr
    geometry_msgs
    tf2
    tf2_eigen
    builtin_interfaces
)

install(
    DIRECTORY include/
    DESTINATION include
)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} # 告诉ros2有这么个目标（可执行文件或者库）
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

ament_package()
# include_directories(${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# #############
# # LIBRARIES #
# #############
# cs_add_library(${PROJECT_NAME}
#   src/kindr_msg.cc
#   src/kindr_tf.cc
#   src/kindr_xml.cc
# )

# ##########
# # GTESTS #
# ##########
# catkin_add_gtest(kindr_tf_test
#   test/kindr_tf_test.cc
# )
# target_link_libraries(kindr_tf_test ${PROJECT_NAME})

# catkin_add_gtest(kindr_msg_test
#   test/kindr_msg_test.cc
# )
# target_link_libraries(kindr_msg_test ${PROJECT_NAME})

# ##########
# # EXPORT #
# ##########
# cs_install()
# cs_export()
