
# yaml-cpp 0.7.0
find_package(yaml-cpp 0.7 REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})
list(APPEND thirdpartlib yaml-cpp)
execute_process(COMMAND "pkg-config" "--modversion" "yaml-cpp" OUTPUT_VARIABLE YAML_CPP_VERSION)
message("yaml-cpp已安装 版本:${YAML_CPP_VERSION}")

# opencv 
# find_package (OpenCV REQUIRED)
# include_directories (${OpenCV_INCLUDE_DIRS}
# target_link_libraries (${thirdpartlib} ${OpenCV_LIBS})
