cmake_minimum_required(VERSION 2.8.3)
project(lqr)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(catkin_deps
        roscpp
        )

find_package(catkin REQUIRED
        COMPONENTS
        ${catkin_deps}
        )

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS ${catkin_deps}
        #  CATKIN_DEPENDS other_catkin_pkg
        #  DEPENDS system_lib
)

include_directories(
        include
        #include/shawn
        ${catkin_INCLUDE_DIRS}
)

add_executable(lqr
        include/matrix.h
        include/ExtendedLQR.h
        src/ExtendedLQR_diffdrive_noTimeOpt.cpp
        src/LQR.cpp)

target_link_libraries(lqr
        ${catkin_LIBRARIES}
        glog
        )