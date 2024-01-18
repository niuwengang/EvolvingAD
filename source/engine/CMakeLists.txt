#ros 配置
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ublox_msgs
)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS} )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)#编译出的可执行文件
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)#编译过程中的lib




add_executable(node test_node.cpp)
target_link_libraries(node ${catkin_LIBRARIES} )#${PROJECT_NAME}.glins_pipe
