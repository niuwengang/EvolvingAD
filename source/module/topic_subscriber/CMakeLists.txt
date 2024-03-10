
include_directories(${catkin_INCLUDE_DIRS} )

add_library(${firstpartylib}.fix_gnss_sub SHARED fix_gnss_sub.cpp )
target_link_libraries(${firstpartylib}.fix_gnss_sub ${catkin_LIBRARIES}  ${thirdpartylib})

add_library(${firstpartylib}.cloud_sub SHARED cloud_sub.cpp )
target_link_libraries(${firstpartylib}.cloud_sub ${catkin_LIBRARIES}  ${thirdpartylib})


