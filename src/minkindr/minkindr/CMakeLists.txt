cmake_minimum_required(VERSION 3.5)
project(minkindr)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake_auto REQUIRED)
find_package(Threads)

ament_auto_find_build_dependencies ()

ament_auto_package ()
# include_directories(${Eigen_INCLUDE_DIRS} ${ament_INCLUDE_DIRS})

# catkin_add_gtest(${PROJECT_NAME}_tests 
#   test/test-main.cc
#   test/test-rotation.cc
#   test/test-sim3.cc
#   test/test-transformation.cc
# )
# target_link_libraries(${PROJECT_NAME}_tests ${catkin_LIBRARIES} 
#                                             ${CMAKE_THREAD_LIBS_INIT})

# # Add all files to show up in QtCreator.
# FILE(GLOB_RECURSE LibFiles "include/*")
# add_custom_target(headers SOURCES ${LibFiles})

# cs_install()
# cs_export()

