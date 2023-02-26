add_executable(position_control PositionControl.cpp)
target_link_libraries(position_control Eigen3::Eigen)
target_link_libraries(position_control ${catkin_LIBRARIES} PID)
