cmake_minimum_required(VERSION 3.19)
project(obstacle_avoidance)

set(CMAKE_CXX_STANDARD 14)
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

find_package(PCL 1.2 REQUIRED)


include_directories(
        ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(obstacle_avoidance main.cpp ProcessPointClouds.cpp ProcessPointClouds.h Box.h CollisionDetection.cpp CollisionDetection.h)

target_link_libraries (obstacle_avoidance ${PCL_LIBRARIES})