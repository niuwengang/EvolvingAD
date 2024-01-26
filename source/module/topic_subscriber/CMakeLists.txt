find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ublox_msgs
)
include_directories(${catkin_INCLUDE_DIRS} )
add_library(${firstpartylib}.imu_sub SHARED imu_sub.cpp)
