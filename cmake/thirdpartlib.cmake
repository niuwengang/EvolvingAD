# yaml-cpp tag:0.70 
find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})
target_link_libraries(${thirdpartlib} yaml-cpp)