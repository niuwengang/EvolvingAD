
# yaml-cpp 0.7.0
find_package(yaml-cpp 0.7 REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})
list(APPEND thirdpartylib yaml-cpp)
execute_process(COMMAND "pkg-config" "--modversion" "yaml-cpp" OUTPUT_VARIABLE YAML_CPP_VERSION)
message("yaml-cpp已安装 版本:${YAML_CPP_VERSION}")

# opencv 4.2.0
find_package (OpenCV REQUIRED)
include_directories (${OpenCV_INCLUDE_DIRS})
list(APPEND thirdpartylib  ${OpenCV_LIBS})
execute_process(COMMAND "pkg-config" "--modversion" "opencv4" OUTPUT_VARIABLE OPENCV4_VERSION)
message("opencv4已安装 版本:${OPENCV4_VERSION}")

# pcl 
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
list(APPEND thirdpartylib  ${PCL_LIBRARIES})


#ros noetic
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ublox_msgs
)
include_directories(${catkin_INCLUDE_DIRS} )
execute_process(COMMAND "rosversion" "-d"  OUTPUT_VARIABLE ROS_VERSION)
message("Ros已安装 版本:${ROS_VERSION}") 





