cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(test_correspondence_estimation)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (test_correspondence_estimation test_correspondence_estimation.cpp)
target_link_libraries (test_correspondence_estimation ${PCL_LIBRARIES})