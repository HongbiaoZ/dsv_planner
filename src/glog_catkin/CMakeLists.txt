#cmake_minimum_required(VERSION 2.8.3)
#project(glog_catkin)

#find_package(catkin REQUIRED)

#include(ExternalProject)

#file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

#set(VERSION 0.3.4)
#ExternalProject_Add(glog_src
#  DOWNLOAD_COMMAND wget --retry-connrefused --waitretry=1 --timeout=12 --tries 10 https://github.com/google/glog/archive/v${VERSION}.zip
#  UPDATE_COMMAND unzip -d . -o ../v${VERSION}.zip
#  CONFIGURE_COMMAND cd ../glog_src/glog-${VERSION} && ./configure --with-pic
#    --with-gflags=${gflags_catkin_PREFIX}
#    --prefix=${CATKIN_DEVEL_PREFIX}
#  BUILD_COMMAND cd ../glog_src/glog-${VERSION} && make -j 8
#  INSTALL_COMMAND cd ../glog_src/glog-${VERSION} && make install -j 8
#)

#add_library(${PROJECT_NAME} src/dependency_tracker.cc)
#add_dependencies(${PROJECT_NAME} glog_src)
#target_link_libraries(${PROJECT_NAME} ${CATKIN_DEVEL_PREFIX}/lib/libglog${CMAKE_SHARED_LIBRARY_SUFFIX})

cmake_minimum_required(VERSION 2.8.3)
project(glog_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ExternalProject)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

set(VERSION 0.3.4)
ExternalProject_Add(glog_src
  DOWNLOAD_COMMAND wget --retry-connrefused --waitretry=1 --timeout=12 --tries 10 https://github.com/google/glog/archive/v${VERSION}.zip
  UPDATE_COMMAND unzip -d . -o ../v${VERSION}.zip
  CONFIGURE_COMMAND cd ../glog_src/glog-${VERSION} && ./configure --with-pic
    --with-gflags=${gflags_catkin_PREFIX}
    --prefix=${CATKIN_DEVEL_PREFIX}
  BUILD_COMMAND cd ../glog_src/glog-${VERSION} && make -j 8
  INSTALL_COMMAND cd ../glog_src/glog-${VERSION} && make install -j 8
)

cs_add_library(${PROJECT_NAME} src/dependency_tracker.cc)
add_dependencies(${PROJECT_NAME} glog_src)
target_link_libraries(${PROJECT_NAME} ${CATKIN_DEVEL_PREFIX}/lib/libglog${CMAKE_SHARED_LIBRARY_SUFFIX})

cs_install()

cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
  CFG_EXTRAS glog-extras.cmake)
