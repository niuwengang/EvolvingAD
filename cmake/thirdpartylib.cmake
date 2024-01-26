
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


## json
# find_package(PkgConfig REQUIRED)
# # pkg_check_modules(JSONCPP jsoncpp)
# list(APPEND thirdpartlib  ${JSONCPP_LIBRARIES})
# execute_process(COMMAND "pkg-config" "--modversion" "jsoncpp" OUTPUT_VARIABLE JSONCPP_VERSION)
# message("jsoncpp已安装 版本:${JSONCPP_VERSION}")

