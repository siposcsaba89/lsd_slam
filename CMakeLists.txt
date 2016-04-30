cmake_minimum_required(VERSION 3.0)
project(lsdslam)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/bin)

option(BUILD_TEST "Build lsd test" OFF)

add_subdirectory(lsd_slam_core)
if(BUILD_TEST)
	add_subdirectory(lsd_test)
endif()