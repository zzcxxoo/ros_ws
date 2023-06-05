cmake_minimum_required(VERSION 3.0.2)
project(robot_gazebo)

# # Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# # System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# # Generate messages in the 'msg' folder
add_message_files(
  FILES
  bbox.msg
)

# # Generate services in the 'srv' folder
add_service_files(
  FILES
  bboxPerFrame.srv
)

# # Generate actions in the 'action' folder
# add_action_files(
# FILES
# Action1.action
# Action2.action
# )

# # Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(

  # INCLUDE_DIRS include
  # LIBRARIES robot_gazebo
  CATKIN_DEPENDS message_generation std_msgs message_runtime

  # DEPENDS system_lib
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(WalkingPerson SHARED
  src/WalkingPerson.cc
)

add_library(MovingCar SHARED
  src/MovingCar.cc
)

add_library(GetBBox SHARED
  src/GetBBox.cc
)
add_library(RotationPlugin SHARED
  src/RotationPlugin.cc
)

set_target_properties(WalkingPerson PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

set_target_properties(MovingCar PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

set_target_properties(GetBBox PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

set_target_properties(RotationPlugin PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

target_link_libraries(WalkingPerson ${GAZEBO_LIBRARIES})
target_link_libraries(MovingCar ${GAZEBO_LIBRARIES})
target_link_libraries(RotationPlugin ${GAZEBO_LIBRARIES})
target_link_libraries(GetBBox ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# # Install ##
install(TARGETS
  WalkingPerson
  MovingCar
  GetBBox
  RotationPlugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION share/robot_gazebo/lib
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY launch worlds models
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
