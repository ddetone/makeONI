cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(label_sv)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (label_sv label.cpp)
target_link_libraries (label_sv ${PCL_LIBRARIES})



MESSAGE( STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS} )
MESSAGE( STATUS "PCL_LIBRARIES:" ${PCL_LIBRARIES} )
MESSAGE( STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS} )
MESSAGE( STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS} )