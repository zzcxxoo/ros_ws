cmake_minimum_required(VERSION 3.10.0)
project(check_services)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    mobile_platform_msgs
    actionlib
    actionlib_msgs
)
add_executable(${PROJECT_NAME}
    check_services.cc
    job_manager.cc
    map_manager.cc
)

# target_compile_options(${PROJECT_NAME} PRIVATE "-fno-rtti")

target_include_directories(${PROJECT_NAME}
  PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${CATKIN_DEVEL_PREFIX}/include
)
target_link_libraries(${PROJECT_NAME}
    PRIVATE
    pthread 
    common::utils
    robot_manager_mapimgtrans
    robot_manager_osshelper
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
)
