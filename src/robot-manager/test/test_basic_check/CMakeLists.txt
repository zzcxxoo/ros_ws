cmake_minimum_required(VERSION 3.16)
project(test_basic_check)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    mobile_platform_msgs
    topic_tools
)

find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(spdlog REQUIRED)

add_executable(test_basic_check test_basic_check.cpp)
add_executable(test_sub_anytype test_sub_anytype.cpp)

target_link_libraries(test_basic_check PUBLIC
    ${catkin_LIBRARIES}
    Boost::filesystem
    spdlog::spdlog
)

target_include_directories(test_basic_check PRIVATE
    ${catkin_INCLUDE_DIRS}
    ../../common    
)

target_link_libraries(test_sub_anytype PUBLIC
    ${catkin_LIBRARIES}
)

target_include_directories(test_sub_anytype PRIVATE
    ${catkin_INCLUDE_DIRS}
)