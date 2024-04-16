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
include(cmake/g2o.cmake)


#消息定义-----------------------------------------------------------------------------------------------
add_library(user_msg_group  
source/user_msg/gnss_msg.cpp
source/user_msg/imu_msg.cpp
source/user_msg/cloud_msg.cpp
source/user_msg/frame_msg.cpp
source/user_msg/ods_msg.cpp
)
target_link_libraries(user_msg_group 
${catkin_LIBRARIES} 
${thirdpartylib}
) 


#工具箱------------------------------------------------------------------------------------------------
add_library(tools_group  
source/tools/subscriber/gnss_sub.cpp
source/tools/subscriber/imu_sub.cpp
source/tools/subscriber/cloud_sub.cpp
source/tools/subscriber/odom_sub.cpp

source/tools/publisher/odom_pub.cpp
source/tools/publisher/cloud_pub.cpp
source/tools/publisher/imu_pub.cpp
source/tools/publisher/tf_pub.cpp
source/tools/publisher/gnss_pub.cpp
source/tools/publisher/bbx_pub.cpp

source/tools/system_monitor/system_monitor.cpp
source/tools/file_manager/file_manager.cpp
 )
target_link_libraries(tools_group 
${catkin_LIBRARIES}
 ${thirdpartylib}
 user_msg_group
 ) 

#模块-----------------------------------------------------------------------------------------------------
##第三方库之fast_gicp
add_library(thirdparty_fast_gicp   
source/thirdpartylib/fast_gicp/src/lsq_registration.cpp
source/thirdpartylib/fast_gicp/src/fast_gicp.cpp
source/thirdpartylib/fast_gicp/src/fast_gicp_st.cpp
source/thirdpartylib/fast_gicp/src/fast_vgicp)
target_link_libraries(thirdparty_fast_gicp 
 ${thirdpartylib}
 ) 


 add_library(thirdparty_segment_ground  
source/thirdpartylib/linefit_ground_segmentation/src/ground_segmentation.cc 
source/thirdpartylib/linefit_ground_segmentation/src/segment.cc 
source/thirdpartylib/linefit_ground_segmentation/src/bin.cc 
source/thirdpartylib/linefit_ground_segmentation/src/viewer.cc)
target_link_libraries(thirdparty_segment_ground 
 ${thirdpartylib}
 ) 


add_library(module_group  

source/module/cloud_registration/ndt_registration.cpp
source/module/cloud_registration/fast_gicp_registration.cpp

source/module/cloud_filter/voxel_filter.cpp

source/module/gnss_odom/gnss_odom.cpp
source/module/lidar_odom/lidar_odom.cpp

source/module/pose_graph/pose_graph.cpp

source/module/graph_optimizer/g2o/g2o_opter.cpp
 )
target_link_libraries(module_group 
${catkin_LIBRARIES}
 ${thirdpartylib}
 user_msg_group
 thirdparty_fast_gicp
 ) 





# 预处理节点---------------------------------------------------------------------------------------------
add_library(preprocerss_pipe
source/pipe/preprocerss_pipe.cpp)
target_link_libraries(preprocerss_pipe 
${catkin_LIBRARIES} 
tools_group
module_group
)

add_executable(preprocerss_node 
source/exec/preprocerss_node.cpp)
target_link_libraries(preprocerss_node 
 preprocerss_pipe)

# 前端里程计---------------------------------------------------------------------------------------------
add_library(front_end_pipe
source/pipe/front_end_pipe.cpp)
target_link_libraries(front_end_pipe 
${catkin_LIBRARIES} 
tools_group
module_group
)

add_executable(front_end_node 
source/exec/front_end_node.cpp)
target_link_libraries(front_end_node 
 front_end_pipe)


# 后端优化---------------------------------------------------------------------------------------------
add_library(back_end_pipe
source/pipe/back_end_pipe.cpp)
target_link_libraries(back_end_pipe 
${catkin_LIBRARIES} 
tools_group
module_group
)

add_executable(back_end_node 
source/exec/back_end_node.cpp)
target_link_libraries(back_end_node 
back_end_pipe )



# 测试---------------------------------------------------------------------------------------------

add_executable(test_node 
source/exec/test_node.cpp)
target_link_libraries(test_node 
module_group
tools_group 
${catkin_LIBRARIES} )


