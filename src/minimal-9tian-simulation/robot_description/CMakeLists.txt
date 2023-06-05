cmake_minimum_required(VERSION 3.10.0)
project(robot_description)

# # Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -std=c++11")
endif()

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  gazebo_dev
  gazebo_msgs
  urdf
  xacro
  roscpp
  std_msgs
  message_generation
  rospy
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)
find_package(Threads REQUIRED)

# # System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# # Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY sensors/msg
  FILES
  EncoderData.msg
)

# # Generate services in the 'srv' folder
# add_service_files(
# FILES
# Service1.srv
# Service2.srv
# )

# # Generate actions in the 'action' folder
# add_action_files(
# FILES
# Action1.action
# Action2.action
# )

# # Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs # Or other packages containing msgs
)

catkin_package(

  # INCLUDE_DIRS include
  # LIBRARIES cleaningrobot_nrsl
  CATKIN_DEPENDS gazebo_plugins gazebo_ros gazebo_ros_control urdf xacro roscpp std_msgs message_runtime rospy

  # DEPENDS system_lib
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(

  # include
  ${catkin_INCLUDE_DIRS}
)

# # Declare a C++ library
# add_library(${PROJECT_NAME}
# src/${PROJECT_NAME}/cleaningrobot_nrsl.cpp
# )
add_library(gazebo_ros_encoder SHARED sensors/src/gazebo_encoder.cpp)
target_link_libraries(gazebo_ros_encoder ${catkin_LIBRARIES})
add_dependencies(gazebo_ros_encoder ${PROJECT_NAME}_generate_messages_cpp)

# ############
# # Install ##
# ############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# # Mark executable scripts (Python etc.) for installation
# # in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  sensors/scripts/encoder_odom.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  gazebo_ros_encoder
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY config launch meshes rviz sensors urdf
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
