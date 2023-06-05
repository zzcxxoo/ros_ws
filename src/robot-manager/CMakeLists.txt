cmake_minimum_required(VERSION 3.10.0)
project(robot_manager)

find_package(catkin REQUIRED COMPONENTS
roscpp
)

option(BUILD_TESTS "build the tests or not" OFF)

catkin_package()

include_directories(".")

add_subdirectory(behavior)
add_subdirectory(params_changer)
add_subdirectory(check_services)
add_subdirectory(common)
add_subdirectory(velocity_smoother)
add_subdirectory(cleaning_state)
add_subdirectory(basic_check)

if(BUILD_TESTS)
  add_subdirectory(test)
endif()

install(
    DIRECTORY config launch setup scripts systemd health_monitor rviz
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
)

install(
    DIRECTORY scripts
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}
    USE_SOURCE_PERMISSIONS
)

install(
    DIRECTORY velocity_smoother/cfg velocity_smoother/plugins
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/velocity_smoother
    USE_SOURCE_PERMISSIONS
)

