cmake_minimum_required(VERSION 3.0.2)
project(urban_nav)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++17)
add_definitions(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslib
  tf
)


catkin_package()



include_directories(
  ${catkin_INCLUDE_DIRS}
  source
)
set(thirdpartylib "")
include(cmake/thirdpartylib.cmake)






#消息定义---------------------
add_library(user_msg_group  
source/user_msg/gnss_msg.cpp
source/user_msg/imu_msg.cpp
source/user_msg/cloud_msg.cpp
source/user_msg/frame_msg.cpp
)
target_link_libraries(user_msg_group 
${catkin_LIBRARIES} 
${thirdpartylib}
) 


#工具箱---------------------
add_library(tools_group  
source/tools/subscriber/gnss_sub.cpp
source/tools/subscriber/imu_sub.cpp
source/tools/subscriber/cloud_sub.cpp

source/tools/publisher/odom_pub.cpp
source/tools/publisher/cloud_pub.cpp
source/tools/publisher/imu_pub.cpp
source/tools/publisher/tf_pub.cpp
source/tools/publisher/gnss_pub.cpp

source/tools/monitor/monitor.cpp
source/tools/file_manager/file_manager.cpp
 )
target_link_libraries(tools_group 
${catkin_LIBRARIES}
 ${thirdpartylib}
 user_msg_group
 ) 

#模块----------------------------------
add_library(module_group  
source/module/cloud_registration/ndt_registration.cpp
source/module/cloud_registration/fast_gicp_registration.cpp
source/module/cloud_filter/voxel_filter.cpp
 )
target_link_libraries(module_group 
${catkin_LIBRARIES}
 ${thirdpartylib}
 user_msg_group
 fast_gicp
 ) 

add_library(fast_gicp   
source/thirdpartylib/fast_gicp/src/lsq_registration.cpp
source/thirdpartylib/fast_gicp/src/fast_gicp.cpp
source/thirdpartylib/fast_gicp/src/fast_gicp_st.cpp
source/thirdpartylib/fast_gicp/src/fast_vgicp)
target_link_libraries(fast_gicp 
# ${catkin_LIBRARIES}
 ${thirdpartylib}
 ) 

# 执行节点
add_executable(odom_node 
source/exec/odom_node.cpp
source/exec/pipe/odom_pipe.cpp
)

target_link_libraries(odom_node 
${catkin_LIBRARIES} 
tools_group 
user_msg_group
module_group
# linefit_ground_segmentation
${thirdpartylib})


add_executable(test_node 
source/exec/test_node.cpp
)

target_link_libraries(test_node 
${catkin_LIBRARIES} 
${thirdpartylib})


