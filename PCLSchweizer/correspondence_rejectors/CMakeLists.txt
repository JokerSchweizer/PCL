cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(correspondence_rejectors)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (correspondence_rejectors test_correspondence_rejectors.cpp)
target_link_libraries (correspondence_rejectors ${PCL_LIBRARIES})