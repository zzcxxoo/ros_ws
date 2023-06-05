add_library(${PROJECT_NAME}_state_machine
    state_machine.cc
    head_state_idle.cc
    head_state_sleep.cc
    head_state_navigation.cc
    head_state_exception.cc
    head_state_charging.cc
    head_state_update.cc
    head_state_configuration.cc
)
add_library(${PROJECT_NAME}::state_machine ALIAS ${PROJECT_NAME}_state_machine)
target_include_directories(${PROJECT_NAME}_state_machine
    PUBLIC
        ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_state_machine
    PUBLIC
        ${PROJECT_NAME}::task_factory

)

add_executable(state_machine_test
    state_machine_test.cc
)
target_link_libraries(state_machine_test
    PRIVATE
        ${PROJECT_NAME}::state_machine
        ${catkin_LIBRARIES}
)