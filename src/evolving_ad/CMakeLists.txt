cmake_minimum_required(VERSION 3.9.0)
project(evolving_ad)

#compile option
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_compile_options(-W) 

#ros
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs roslib tf)
include_directories(${catkin_INCLUDE_DIRS})
catkin_package()


add_executable(loc_node  source/exec/loc_node.cpp)
target_link_libraries(loc_node ${catkin_LIBRARIES} )