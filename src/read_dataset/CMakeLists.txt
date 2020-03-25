add_executable(read_dataset main.cpp)

target_link_libraries(read_dataset  ${catkin_LIBRARIES})

add_dependencies(read_dataset ${${PROJECT_NAME}_EXPORTED_TARGETS})
