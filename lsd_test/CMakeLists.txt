cmake_minimum_required(VERSION 3.0)

project(test_lsd_slam)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)


if(NOT TARGET ::lsdslam)
	message("INFORMATION Finding lsdslam ... ")
	find_package(lsdslam REQUIRED)
endif()

add_executable(test_lsd_slam main.cpp)
target_link_libraries(test_lsd_slam ::lsdslam)
