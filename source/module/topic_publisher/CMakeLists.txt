
include_directories(${catkin_INCLUDE_DIRS} )


add_library(${firstpartylib}.cloud_pub SHARED cloud_pub.cpp )
target_link_libraries(${firstpartylib}.cloud_pub ${catkin_LIBRARIES}  ${thirdpartylib})

add_library(${firstpartylib}.polygon_pub SHARED polygon_pub.cpp )
target_link_libraries(${firstpartylib}.polygon_pub ${catkin_LIBRARIES}  ${thirdpartylib})
