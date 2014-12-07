cmake_minimum_required(VERSION 2.8)
 
PROJECT(depth_image_visualization)
 
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

include_directories(${VTK_INCLUDE_DIRS})
link_directories(${VTK_LIBRARY_DIRS})
add_definitions(${VTK_DEFINITIONS})
 
add_executable (depth_image_visualization depth_image_visualization.cpp)
 

target_link_libraries (depth_image_visualization ${VTK_LIBRARIES})
