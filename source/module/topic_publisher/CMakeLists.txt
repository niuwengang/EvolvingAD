
include_directories(${catkin_INCLUDE_DIRS} )


add_library(${firstpartylib}.cloud_pub SHARED cloud_pub.cpp )
target_link_libraries(${firstpartylib}.cloud_pub ${catkin_LIBRARIES}  ${thirdpartylib})


