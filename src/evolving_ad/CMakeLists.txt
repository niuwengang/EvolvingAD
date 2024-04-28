cmake_minimum_required(VERSION 3.9.0)
project(evolving_ad)

#compile option
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_compile_options(-W) 

include_directories(source)
set(thirdpartylib "")
include(cmake/thirdpartylib.cmake)

#ros
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs roslib tf)
include_directories(${catkin_INCLUDE_DIRS})
catkin_package()

#msg
aux_source_directory(source/msg   MSG_SRC)
add_library(msg  ${MSG_SRC})
target_link_libraries(msg  ${catkin_LIBRARIES} ${thirdpartylib})

#topic sub
aux_source_directory(source/topic_sub   TOPIC_SUB)
add_library(topic_sub  ${TOPIC_SUB})
target_link_libraries(topic_sub  ${catkin_LIBRARIES} ${thirdpartylib})

#topic pub
aux_source_directory(source/topic_pub   TOPIC_PUB)
add_library(topic_pub  ${TOPIC_PUB})
target_link_libraries(topic_pub  ${catkin_LIBRARIES} ${thirdpartylib})

add_library(front_end source/pipe/front_end.cpp)
target_link_libraries(front_end ${catkin_LIBRARIES} topic_sub  topic_pub msg )

add_executable(ad_node  source/exec/ad_node.cpp)
target_link_libraries(ad_node front_end )