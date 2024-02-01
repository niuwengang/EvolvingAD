
include_directories(${catkin_INCLUDE_DIRS} )
add_library(${firstpartylib}.fix_gnss_sub SHARED fix_gnss_sub.cpp  ${catkin_LIBRARIES} )
