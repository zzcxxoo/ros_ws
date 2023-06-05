cmake_minimum_required(VERSION 3.0.2)
project(cleaning_time)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mobile_platform_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system filesystem)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ../common/
)

add_executable(cleaning_timing cleaning_timing.cpp)
target_link_libraries(cleaning_timing PRIVATE
    ${catkin_LIBRARIES}
)

add_executable(cleaning_task cleaning.cpp)
target_link_libraries(cleaning_task PRIVATE
    ${catkin_LIBRARIES}
    uuid
)

install(TARGETS cleaning_task cleaning_timing
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
)
install(TARGETS cleaning_timing
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
)