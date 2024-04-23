cmake_minimum_required(VERSION 3.9.0)
project(urban_nav)

#compile option
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")


add_compile_options(-W) 
option(USE_CUDA "Enable the use of CUDA" ON)  #if use cuda 


#cuda
if(USE_CUDA)
  enable_language(CUDA)
  set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)
  set(CUDA_INCLUDE_DIRS ${CUDA_TOOLKIT_ROOT_DIR}/${CUDA_INSTALL_TARGET_DIR}/include)
  set( SMS 30 32 35 37 50 52 53 60 61 62 70 72 75 87)
  foreach(sm ${SMS})
	set(GENCODE ${GENCODE} -gencode arch=compute_${sm},code=sm_${sm})
  endforeach()
  set(HIGHEST_SM 87)
  set(GENCODE ${GENCODE} -gencode arch=compute_${HIGHEST_SM},code=compute_${HIGHEST_SM})
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}
    -ccbin ${CMAKE_CXX_COMPILER}
    -Xcompiler -DWIN_INTERFACE_CUSTOM
    -Xcompiler -I/usr/aarch64-linux-gnu/include/
    -Xlinker -lsocket
    -Xlinker -rpath=/usr/lib/aarch64-linux-gnu/
    -Xlinker -rpath=/usr/aarch64-linux-gnu/lib/
    -Xlinker -L/usr/lib/aarch64-linux-gnu/
    -Xlinker -L/usr/aarch64-linux-gnu/lib/)
  find_package(CUDA REQUIRED)
  include_directories(${CUDA_INCLUDE_DIRS}  ${TENSORRT_INCLUDE_DIRS} )
endif()

#ros
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs roslib tf)
include_directories(${catkin_INCLUDE_DIRS})
catkin_package()



include_directories(source)

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
aux_source_directory(source/thirdpartylib/fast_gicp/src  THIRDPARTY_FAST_GICP_SRC)
add_library(thirdparty_fast_gicp   ${THIRDPARTY_FAST_GICP_SRC})
target_link_libraries(thirdparty_fast_gicp ${thirdpartylib} ) 

##thirdparty--pointpillar
if(USE_CUDA)
  aux_source_directory(source/thirdpartylib/pointpillar/src   THIRDPARTY_POINT_PILLAR_SRC)
  cuda_add_library(thirdparty_pointpillar   ${THIRDPARTY_POINT_PILLAR_SRC})
  target_link_libraries(thirdparty_pointpillar   libnvinfer.so libnvonnxparser.so) 
endif()

##thirdparty--ground_seg
# aux_source_directory(source/thirdpartylib/ground_seg/src   THIRDPARTY_GROUND_SEG_SRC)
# add_library(thirdparty_ground_seg   ${THIRDPARTY_GROUND_SEG_SRC})
# target_link_libraries(thirdparty_ground_seg ${thirdpartylib} ) 


#module_group--------------------------------------------------------------------------------------------
add_library(module_group  
source/module/cloud_registration/ndt_registration.cpp
source/module/cloud_registration/fast_gicp_registration.cpp
source/module/cloud_filter/voxel_filter.cpp
source/module/gnss_odom/gnss_odom.cpp
source/module/lidar_odom/lidar_odom.cpp
source/module/pose_graph/pose_graph.cpp
source/module/graph_optimizer/g2o/g2o_opter.cpp
source/module/object_detection/object_detection.cpp
source/module/ground_segement/dipg_ground_segement.cpp
)
target_link_libraries(module_group ${catkin_LIBRARIES} ${thirdpartylib} 
user_msg_group thirdparty_fast_gicp thirdparty_pointpillar ) 


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


# # back end---------------------------------------------------------------------------------------------
# add_library(back_end_pipe source/pipe/back_end_pipe.cpp)
# target_link_libraries(back_end_pipe  ${catkin_LIBRARIES}  tools_group module_group)
# add_executable(back_end_node  source/exec/back_end_node.cpp)
# target_link_libraries(back_end_node back_end_pipe )

#test---------------------------------------------------------------------------------------------
# add_executable(test_node source/exec/test_node.cpp)
# target_link_libraries(test_node  ${catkin_LIBRARIES}  tools_group   module_group )