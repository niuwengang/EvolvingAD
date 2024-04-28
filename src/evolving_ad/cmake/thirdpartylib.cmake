
# yaml-cpp 0.7.0
find_package(yaml-cpp 0.7 REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})
list(APPEND thirdpartylib yaml-cpp)

# opencv 4.2.0
find_package (OpenCV REQUIRED)
include_directories (${OpenCV_INCLUDE_DIRS})
list(APPEND thirdpartylib  ${OpenCV_LIBS})

# pcl 
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
list(APPEND thirdpartylib  ${PCL_LIBRARIES})

#geographiclib 
find_package (GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIRS})
list(APPEND thirdpartylib  ${GeographicLib_LIBRARIES})

# #openmp no need install 
# find_package(OpenMP REQUIRED)
# list(APPEND thirdpartylib  OpenMP::OpenMP_CXX)





