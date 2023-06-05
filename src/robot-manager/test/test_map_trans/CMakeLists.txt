cmake_minimum_required(VERSION 3.10.0)
project(test_timing_task)

add_executable(test test.cpp)

target_include_directories(test PRIVATE
	../../common
	)

target_link_directories(test PRIVATE
	../../../../devel/lib
	)

target_link_libraries(test
	robot_manager_mapimgtrans
	)