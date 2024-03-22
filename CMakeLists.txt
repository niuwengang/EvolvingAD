#编译配置
cmake_minimum_required(VERSION 3.15)
project(ad_common_project)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)#编译出的可执行文件
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)#编译过程中的lib
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)


set(firstpartylib "firstpartylib")#相当于全局变量
set(thirdpartylib "")

include(cmake/thirdpartylib.cmake)
include_directories(source)#头文件从source开始找

add_subdirectory(source)

