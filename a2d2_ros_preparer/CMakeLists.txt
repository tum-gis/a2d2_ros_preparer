cmake_minimum_required(VERSION 3.0.2)
project(a2d2_ros_preparer)

set(EXECUTABLE_NAME converter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PACKAGE_DEPENDENCIES
        pcl_conversions
        rosbag
        roscpp
        roslib
        tf2
        tf2_eigen
        tf2_ros
        eigen_conversions
        sensor_msgs
        nav_msgs
        cv_bridge
        image_transport
)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
include_directories(${catkin_INCLUDE_DIRS})

find_package(TBB QUIET)
find_package(CNPY REQUIRED)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES turtlebot_nav
        #  CATKIN_DEPENDS other_catkin_pkg
        #  DEPENDS system_lib
)

file(GLOB_RECURSE ALL_SRCS "src/*.cpp" "src/*.h")
file(GLOB_RECURSE ALL_TESTS "src/*_test.cpp")
file(GLOB_RECURSE ALL_EXECUTABLES "src/*_main.cpp")
#list(REMOVE_ITEM ALL_SRCS ${ALL_TESTS})
list(REMOVE_ITEM ALL_SRCS ${ALL_EXECUTABLES})
add_library(${PROJECT_NAME} STATIC ${ALL_SRCS})

#add_library(${PROJECT_NAME} STATIC ${ALL_SRCS})

add_executable(${EXECUTABLE_NAME} src/converter_main.cpp ${ALL_SRCS})


target_link_libraries(${EXECUTABLE_NAME} PUBLIC tbb)

target_link_libraries(${EXECUTABLE_NAME} PUBLIC ${catkin_LIBRARIES})

# CNPY
target_link_libraries(${EXECUTABLE_NAME} PUBLIC cnpy::cnpy stdc++fs)

# Glog
if (WIN32)
    find_package(glog REQUIRED)
    set(GLOG_LIBRARY glog::glog)
else()
    set(GLOG_LIBRARY glog)
endif()
target_link_libraries(${EXECUTABLE_NAME} PUBLIC ${GLOG_LIBRARY})

install(TARGETS ${EXECUTABLE_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config rviz urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
