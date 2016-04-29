cmake_minimum_required(VERSION 3.0)

project(test_lsd_slam)

find_package(lsdslam REQUIRED)

add_executable(test_lsd_slam main.cpp)
target_link_libraries(test_lsd_slam ::lsdslam)
