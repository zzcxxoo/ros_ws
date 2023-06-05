cmake_minimum_required(VERSION 3.10.0)
project(robot_control_center)

include_directories(".")


find_package(catkin REQUIRED COMPONENTS
    roscpp
)

catkin_package()

add_executable(${PROJECT_NAME} main.cc)
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${catkin_INCLUDE_DIRS}
)

add_subdirectory(proto)
add_subdirectory(state_machine)
add_subdirectory(task_factory)
