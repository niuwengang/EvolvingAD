cmake_minimum_required(VERSION 3.0.2)
project(urban_nav)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++17)
add_definitions(-std=c++17)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs roslib tf)


catkin_package()

include_directories(${catkin_INCLUDE_DIRS} source)

set(thirdpartylib "")
include(cmake/thirdpartylib.cmake)
include(cmake/g2o.cmake)


#msg group-----------------------------------------------------------------------------------------------
aux_source_directory(source/user_msg USER_MSG_SRC)
add_library(user_msg_group  ${USER_MSG_SRC})
target_link_libraries(user_msg_group  ${catkin_LIBRARIES}  ${thirdpartylib}) 


#tools group------------------------------------------------------------------------------------------------
aux_source_directory(source/tools/subscriber      TOOLS_SUBSCRIBER_SRC)
aux_source_directory(source/tools/publisher       TOOLS_PUBLISHER_SRC)
aux_source_directory(source/tools/system_monitor  TOOLS_SYSTEM_MONITOR_SRC)
aux_source_directory(source/tools/file_manager    TOOLS_FILE_MANAGER_SRC)
add_library(tools_group   ${TOOLS_SUBSCRIBER_SRC} ${TOOLS_PUBLISHER_SRC} ${TOOLS_SYSTEM_MONITOR_SRC} ${TOOLS_FILE_MANAGER_SRC})
target_link_libraries(tools_group  ${catkin_LIBRARIES} ${thirdpartylib} user_msg_group) 


#thirdparty--------------------------------------------------------------------------------------------
##thirdparty--fasticp
aux_source_directory(source/thirdpartylib/fast_gicp/src      THIRDPARTY_FAST_GICP_SRC)
add_library(thirdparty_fast_gicp   ${THIRDPARTY_FAST_GICP_SRC})
target_link_libraries(thirdparty_fast_gicp ${thirdpartylib} ) 


#module_group--------------------------------------------------------------------------------------------
add_library(module_group  
source/module/cloud_registration/ndt_registration.cpp
source/module/cloud_registration/fast_gicp_registration.cpp
source/module/cloud_filter/voxel_filter.cpp
source/module/gnss_odom/gnss_odom.cpp
source/module/lidar_odom/lidar_odom.cpp
source/module/pose_graph/pose_graph.cpp
source/module/graph_optimizer/g2o/g2o_opter.cpp)
target_link_libraries(module_group ${catkin_LIBRARIES} ${thirdpartylib} user_msg_group thirdparty_fast_gicp) 


# preprocess ---------------------------------------------------------------------------------------------
add_library(preprocerss_pipe source/pipe/preprocerss_pipe.cpp )
target_link_libraries(preprocerss_pipe  ${catkin_LIBRARIES}  tools_group module_group)
add_executable(preprocerss_node source/exec/preprocerss_node.cpp )
target_link_libraries(preprocerss_node  preprocerss_pipe)

# front end---------------------------------------------------------------------------------------------
add_library(front_end_pipe source/pipe/front_end_pipe.cpp )
target_link_libraries(front_end_pipe  ${catkin_LIBRARIES}  tools_group module_group)
add_executable(front_end_node source/exec/front_end_node.cpp )
target_link_libraries(front_end_node  front_end_pipe)


# back end---------------------------------------------------------------------------------------------
add_library(back_end_pipe source/pipe/back_end_pipe.cpp)
target_link_libraries(back_end_pipe  ${catkin_LIBRARIES}  tools_group module_group)
add_executable(back_end_node  source/exec/back_end_node.cpp)
target_link_libraries(back_end_node back_end_pipe )



