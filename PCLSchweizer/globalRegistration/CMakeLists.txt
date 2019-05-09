cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(GlobalRegistration)

find_package(PCL 1.6 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (globalRegistration globalRegistration.cpp)
target_link_libraries (globalRegistration ${PCL_LIBRARIES})