project(robot_manager_common)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    mobile_platform_msgs
)
find_package(OpenSSL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem)

set(Jsoncpp_INCLUDE_PATH "/usr/include/jsoncpp")
set(Jsoncpp_LIBRARY "/usr/lib/x86_64-linux-gnu/libjsoncpp.so")

add_library(${PROJECT_NAME} "")
add_library(common::utils ALIAS ${PROJECT_NAME})
add_library(robot_manager_mapimgtrans MapImageTrans.cpp)
add_library(robot_manager_osshelper OssUploadHelper.cpp)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${YAML_CPP_LIBRARIES}
    ${Jsoncpp_LIBRARY}
    ${catkin_LIBRARIES}
    alibabacloud-oss-cpp-sdk
    curl
    crypto
    ssl
    Boost::filesystem
    uuid
)

target_include_directories(robot_manager_mapimgtrans PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${YAML_CPP_LIBRARIES}
)

target_include_directories(robot_manager_osshelper PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(robot_manager_osshelper PRIVATE
    ${catkin_LIBRARIES}
    alibabacloud-oss-cpp-sdk
    curl
    crypto
    ssl
    Boost::filesystem
    uuid
)

target_link_libraries(robot_manager_mapimgtrans PRIVATE 
    ${OpenCV_LIBS}
    ${YAML_CPP_LIBRARIES}    
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${YAML_CPP_INCLUDE_DIR}
    ${Jsoncpp_INCLUDE_PATH}
    ${catkin_INCLUDE_DIRS}
    ${CATKIN_DEVEL_PREFIX}/include
)

target_sources(${PROJECT_NAME}
    PRIVATE
    common.cc
)
message(STATUS "********************* this is the PREFIX" ${CATKIN_INSTALL_PREFIX})
message(STATUS "********************* this is the way " ${CATKIN_PACKAGE_LIB_DESTINATION})
message(STATUS "********************* this is the way " ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(cmd_vel_to_drive_command cmd_vel_to_drive_command.cpp)
target_include_directories(cmd_vel_to_drive_command PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(cmd_vel_to_drive_command ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS robot_manager_mapimgtrans robot_manager_osshelper
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS cmd_vel_to_drive_command
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
)