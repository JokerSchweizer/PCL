cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(SAC_AI_ICP)

find_package(PCL 1.6 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (SAC_AI_ICP SAC_AI_ICP.cpp)
target_link_libraries (SAC_AI_ICP ${PCL_LIBRARIES})