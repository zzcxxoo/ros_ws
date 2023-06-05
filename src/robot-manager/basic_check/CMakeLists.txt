cmake_minimum_required(VERSION 3.16)
project(basic_check)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    mobile_platform_msgs
    topic_tools
)

find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(spdlog REQUIRED)

add_executable(basic_check basic_check.cpp)

target_link_libraries(basic_check PUBLIC
    ${catkin_LIBRARIES}
    Boost::filesystem
    spdlog::spdlog
)

target_include_directories(basic_check PRIVATE
    ${catkin_INCLUDE_DIRS}
    ../
)

install(TARGETS basic_check
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
)