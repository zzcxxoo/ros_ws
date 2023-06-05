project(params_changer)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    mobile_platform_msgs
)

add_executable(${PROJECT_NAME}
    params_changer.cc
    param_manager.cc
)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${CATKIN_DEVEL_PREFIX}/include
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    pthread
    common::utils
    robot_manager_mapimgtrans
    robot_manager_osshelper
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
)
