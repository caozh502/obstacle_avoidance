cmake_minimum_required(VERSION 3.19)
project(pcfilter)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcfilter main.cpp)
target_link_libraries (pcfilter ${PCL_LIBRARIES})