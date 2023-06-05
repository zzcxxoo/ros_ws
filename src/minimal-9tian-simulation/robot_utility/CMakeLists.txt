cmake_minimum_required(VERSION 3.0.2)
project(robot_utility)

# # Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(

  # INCLUDE_DIRS include
  # LIBRARIES robot_utility
  # CATKIN_DEPENDS roscpp rospy std_msgs
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

# # Declare a C++ executable
# # With catkin_make all packages are built within a single CMake context
# # The recommended prefix ensures that target names across packages don't collide
add_executable(fromP3dToTF src/fromP3dToTF.cpp)
target_link_libraries(fromP3dToTF
  ${catkin_LIBRARIES}
)

add_executable(remap_topic_node src/remapTopic.cpp)
target_link_libraries(remap_topic_node
  ${catkin_LIBRARIES}
)
# # Install ##
install(TARGETS 
  fromP3dToTF
  remap_topic_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
