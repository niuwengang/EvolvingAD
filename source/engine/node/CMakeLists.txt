project(node)
#ros 配置
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ublox_msgs
)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS} )

add_executable(node test_node.cpp)
target_link_libraries(node ${catkin_LIBRARIES} )#${PROJECT_NAME}.glins_pipe
