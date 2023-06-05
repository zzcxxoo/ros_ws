cmake_minimum_required(VERSION 3.16)
project(test_oss)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    mobile_platform_msgs
)

find_package(Boost REQUIRED COMPONENTS filesystem)

add_executable(test_oss test_oss_upload.cpp)

target_link_libraries(test_oss PUBLIC
    ${catkin_LIBRARIES}
    alibabacloud-oss-cpp-sdk
    curl
    crypto
    ssl
    Boost::filesystem
    yaml-cpp
    uuid
)

target_include_directories(test_oss PRIVATE
    ${catkin_INCLUDE_DIRS}
    ../../common    
)

install(TARGETS test_oss
    RUNTIME DESTINATION /home/hgy/9tian_ws/devel/lib/robot_manager/
)

