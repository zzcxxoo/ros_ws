project(behavior)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    actionlib
    mobile_platform_msgs
)

add_executable(${PROJECT_NAME}
  behavior.cc
  waypoint_manager.cc
  status_report.cc
)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${CATKIN_DEVEL_PREFIX}/include
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  common::utils
)

link_directories(/usr/local/lib)
include_directories(/usr/local/include/yaml-cpp)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
)


