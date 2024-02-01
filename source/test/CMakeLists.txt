add_executable(opencv_test opencv_test.cpp)
target_link_libraries(opencv_test ${thirdpartylib})

add_executable(yaml_test yaml_test.cpp)
target_link_libraries(yaml_test ${thirdpartylib})


add_executable(topic_test topic_test.cpp)
target_link_libraries(topic_test ${thirdpartylib} ${firstpartylib}.fix_gnss_sub ${catkin_LIBRARIES})





