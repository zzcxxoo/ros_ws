cmake_minimum_required(VERSION 3.0.2)
project(cleaning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE Debug)
set(LOG_CONSOLE OFF)

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
  include/JtcxCommonTools
  include/JtcxCommonDefinitions
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${nlohmann_json_INCLUDE_DIRS}
)

AUX_SOURCE_DIRECTORY(src/JtcxCommonTools SRC1)

add_executable(cleaning_task ./src/RobotCleaningState.cpp ${SRC1})
target_link_libraries(cleaning_task PRIVATE
    ${catkin_LIBRARIES}
    uuid
)

install(TARGETS cleaning_task
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
)
