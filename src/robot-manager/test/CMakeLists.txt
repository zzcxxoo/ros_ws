cmake_minimum_required(VERSION 3.10.0)
project(test_examples)

find_package(catkin REQUIRED COMPONENTS
    roscpp
)

add_executable(${PROJECT_NAME}
  simple_example.cc
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
