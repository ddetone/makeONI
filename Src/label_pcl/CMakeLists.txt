cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(example_supervoxels)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (example_supervoxels example_supervoxels.cpp)
target_link_libraries (example_supervoxels ${PCL_LIBRARIES})



MESSAGE( STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS} )
MESSAGE( STATUS "PCL_LIBRARIES:" ${PCL_LIBRARIES} )
MESSAGE( STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS} )
MESSAGE( STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS} )