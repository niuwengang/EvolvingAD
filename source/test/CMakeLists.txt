add_executable(opencv_test opencv_test.cpp)
target_link_libraries(opencv_test ${thirdpartylib})

add_executable(yaml_test yaml_test.cpp)
target_link_libraries(yaml_test ${thirdpartylib})


add_executable(json_test json_test.cpp)
target_link_libraries(json_test ${thirdpartylib})





