add_executable(lane_localmap node/node.cpp)
target_link_libraries(lane_localmap ${catkin_LIBRARIES} )#${PROJECT_NAME}.glins_pipe
