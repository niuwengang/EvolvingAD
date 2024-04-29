cmake_minimum_required(VERSION 3.9.0)
project(evolving_ad)



#compile option
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_compile_options(-W) 


#cuda
option(USE_CUDA "Enable the use of CUDA" ON)  #if use cuda 
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


#ros--------------------------------------------
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs roslib tf)
include_directories(${catkin_INCLUDE_DIRS})
catkin_package()

include_directories(source)
include_directories(./)
set(thirdpartylib "")
include(cmake/thirdpartylib.cmake)


#msg--------------------------------------------
aux_source_directory(source/msg   MSG_SRC)
add_library(msg  ${MSG_SRC})
target_link_libraries(msg  ${catkin_LIBRARIES} ${thirdpartylib})

#topic--------------------------------------------
aux_source_directory(source/topic_sub   TOPIC_SUB)
add_library(topic_sub  ${TOPIC_SUB})
target_link_libraries(topic_sub  ${catkin_LIBRARIES} ${thirdpartylib})

aux_source_directory(source/topic_pub   TOPIC_PUB)
add_library(topic_pub  ${TOPIC_PUB})
target_link_libraries(topic_pub  ${catkin_LIBRARIES} ${thirdpartylib})

#thirdparty--pointpillar--------------------------------------------
if(USE_CUDA)
  aux_source_directory(thirdpartylib/pointpillar/src   THIRDPARTY_POINT_PILLAR_SRC)
  cuda_add_library(thirdparty_pointpillar   ${THIRDPARTY_POINT_PILLAR_SRC})
  target_link_libraries(thirdparty_pointpillar   libnvinfer.so libnvonnxparser.so) 
endif()

##thirdparty--fasticp--------------------------------------------
aux_source_directory(thirdpartylib/fast_gicp/src  THIRDPARTY_FAST_GICP_SRC)
add_library(thirdparty_fast_gicp   ${THIRDPARTY_FAST_GICP_SRC})
target_link_libraries(thirdparty_fast_gicp ${thirdpartylib} ) 

#tools--------------------------------------------
aux_source_directory(source/tools TOOLS_SRC)
add_library(tools ${TOOLS_SRC})
target_link_libraries(tools ${thirdpartylib} ) 

#module--------------------------------------------
file(GLOB_RECURSE MODULE_SRC "source/module/*.cpp")
add_library(module ${MODULE_SRC})
target_link_libraries(module ${catkin_LIBRARIES} ${thirdpartylib} 
msg 
thirdparty_pointpillar 
thirdparty_fast_gicp ) 


#pipe--------------------------------------------
aux_source_directory(source/pipe  PIPE_SRC)
add_library(pipe ${PIPE_SRC})
target_link_libraries(pipe ${catkin_LIBRARIES} 
topic_sub  
topic_pub 
module 
msg 
tools)

#exec--------------------------------------------
add_executable(ad_node  source/exec/ad_node.cpp)
target_link_libraries(ad_node pipe )

