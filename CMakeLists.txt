cmake_minimum_required(VERSION 3.15)
project(PointCloudHough)

set(CMAKE_CXX_STANDARD 14)

find_package( OpenCV 3 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(PCL 1.8 REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
add_definitions( -DPCL_NO_PRECOMPILE )
link_directories( ${PCL_LIBRARY_DIRS} )

add_executable(PointCloudHough main.cpp HOUGH_LINE.h HOUGH_LINE.cpp DBSCAN.h fitting.cpp fitting.h)
add_executable(txt_converter txt_converter.cpp)
add_executable(PCDRead PCD_Reader.cpp)
add_executable(Visualize VisualizePCD.cpp)
add_executable(OLS OLS.cpp fitting.cpp fitting.h)
add_executable(OLS_double OLS_double.cpp fitting.cpp fitting.h)


target_link_libraries( PointCloudHough ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_link_libraries( txt_converter ${PCL_LIBRARIES})
target_link_libraries( PCDRead ${PCL_LIBRARIES})
target_link_libraries( Visualize ${PCL_LIBRARIES})
target_link_libraries( OLS ${PCL_LIBRARIES})
target_link_libraries( OLS_double ${PCL_LIBRARIES})
