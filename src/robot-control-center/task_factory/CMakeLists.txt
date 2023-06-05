add_library(${PROJECT_NAME}_task_factory
    base_task_factory.cc
    main_task.cc
)
add_library(${PROJECT_NAME}::task_factory ALIAS ${PROJECT_NAME}_task_factory)
target_include_directories(${PROJECT_NAME}_task_factory
    PRIVATE
        ${catkin_INCLUDE_DIRS}
    PUBLIC
        ${CMAKE_BINARY_DIR}
)
target_link_libraries(${PROJECT_NAME}_task_factory
    PUBLIC
        ${PROJECT_NAME}::proto
)


add_executable(task_factory_test
    task_factory_test.cc
)
target_include_directories(task_factory_test
    PRIVATE
        ${catkin_INCLUDE_DIRS}
)
target_link_libraries(task_factory_test
    PRIVATE 
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}::task_factory
)

add_executable(aux_tasks_test
    test_aux_tasks.cc
)
