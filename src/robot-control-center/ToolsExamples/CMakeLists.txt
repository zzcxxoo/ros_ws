cmake_minimum_required(VERSION 2.6)

project(Test)

SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11")
find_package(Boost REQUIRED COMPONENTS filesystem )
include_directories(${Boost_INCLUDE_DIRS})
message("${Boost_INCLUDE_DIRS}")

set(HomePath ${CMAKE_CURRENT_SOURCE_DIR}/../)
include_directories(${HomePath}/common/include/JtcxCommonTools)
include_directories(${HomePath}/common/include/JtcxCommonDefinitions)
AUX_SOURCE_DIRECTORY(${HomePath}/common/src/JtcxCommonTools SRC1)

add_executable(Test ./example.cc ${SRC1})
target_link_libraries(Test pthread ${Boost_LIBRARIES})
