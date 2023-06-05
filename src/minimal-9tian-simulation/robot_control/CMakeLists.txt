cmake_minimum_required(VERSION 3.0.2)
project(robot_control)

# # Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
)

catkin_package(

  # INCLUDE_DIRS include
  # LIBRARIES robot_control
  # CATKIN_DEPENDS controller_manager joint_state_controller robot_state_controller
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

# # Install ##
install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
