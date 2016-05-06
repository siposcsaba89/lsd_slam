cmake_minimum_required(VERSION 2.8.7)
project(lsd_slam_core)

if(MSVC)
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}" "/MP")
      MESSAGE(STATUS "Added parallel build arguments to CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
endif()

set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Release)

#rosbuild_init() 

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#set(CMAKE_MODULE_PATH   ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

find_package(Eigen3 REQUIRED)
message(INFORMATION " eigen version ${EIGEN3_VERSION_STRING}")
#find_package(SuiteParse REQUIRED) # Apparently needed by g2o
find_package(OpenCV REQUIRED)
message(INFORMATION " opencv version ${OpenCV_VERSION}")

if (NOT TARGET csparse)
	find_package(csparse REQUIRED)
endif()

message(INFORMATION " csparse version ${csparse_VERSION}")


set(Boost_USE_STATIC_LIBS        ON) # only find static libs
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME    OFF)
find_package(Boost REQUIRED COMPONENTS thread chrono)
message(INFORMATION " boost version ${Boost_VERSION}")

find_package(G2O REQUIRED)
message(INFORMATION " g2o version ${G2O_VERSION}")

#message(FATAL_ERROR ${G2O_LIBRARIES} ${G2O_INCLUDE_DIRS})
#find_package(X11 REQUIRED)

# FabMap
# uncomment this part to enable fabmap
#add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap)
#include_directories(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap/include)
#add_definitions("-DHAVE_FABMAP")
#set(FABMAP_LIB openFABMAP )

# Dynamic Reconfigure Services
#rosbuild_find_ros_package(dynamic_reconfigure)
#include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
#gencfg()

# SSE flags
#rosbuild_check_for_sse()

# Also add some useful compiler flag
if(NOT MSVC)
	set(CMAKE_CXX_FLAGS
		"${CMAKE_CXX_FLAGS} ${SSE_FLAGS} -march=native  -std=c++0x"
	)
endif()	

# Set source files
set(lsd_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/DataStructures/Frame.cpp
  ${PROJECT_SOURCE_DIR}/src/DataStructures/FramePoseStruct.cpp
  ${PROJECT_SOURCE_DIR}/src/DataStructures/FrameMemory.cpp
  ${PROJECT_SOURCE_DIR}/src/SlamSystem.cpp
  ${PROJECT_SOURCE_DIR}/src/LiveSLAMWrapper.cpp
  ${PROJECT_SOURCE_DIR}/src/DepthEstimation/DepthMap.cpp
  ${PROJECT_SOURCE_DIR}/src/DepthEstimation/DepthMapPixelHypothesis.cpp
  ${PROJECT_SOURCE_DIR}/src/util/globalFuncs.cpp
  ${PROJECT_SOURCE_DIR}/src/util/SophusUtil.cpp
  ${PROJECT_SOURCE_DIR}/src/util/settings.cpp
  ${PROJECT_SOURCE_DIR}/src/util/Undistorter.cpp
  ${PROJECT_SOURCE_DIR}/src/util/gettimeofday.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/Sim3Tracker.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/Relocalizer.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/SE3Tracker.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/TrackingReference.cpp
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/Timestamp.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/FabMap.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/KeyFrameGraph.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/g2oTypeSim3Sophus.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/TrackableKeyFrameSearch.cpp
)
set(SOURCE_FILES
  ${lsd_SOURCE_FILES}
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/ROS/ROSImageStreamThread.cpp
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/ROS/ROSOutput3DWrapper.cpp
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
)


# build shared library.
add_library(lsdslam SHARED ${SOURCE_FILES})
add_library(::lsdslam ALIAS lsdslam)
target_compile_definitions(lsdslam PUBLIC -DENABLE_SSE -DBOOST_ALL_NO_LIB)

#g2o_core g2o_stuff csparse cxsparse g2o_solver_csparse g2o_csparse_extension g2o_types_sim3 g2o_types_sba
target_link_libraries(lsdslam 
	${FABMAP_LIB} 
	${OpenCV_LIBS} 
	${Boost_LIBRARIES}
	${G2O_LIBRARIES}
	::csparse
)
target_include_directories(lsdslam PUBLIC 
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:include/lsdslam>
	${OpenCV_INCLUDE_DIRS} 
	${Boost_INCLUDE_DIRS}
	${G2O_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/thirdparty/Sophus>
	$<INSTALL_INTERFACE:include/lsdslam/Sophus>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/>
)

include(GenerateExportHeader)
generate_export_header(lsdslam)

install(FILES 
		${CMAKE_CURRENT_BINARY_DIR}/lsdslam_export.h
	DESTINATION
		include/lsdslam/
)

install(TARGETS lsdslam EXPORT lsdslam-targets
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

install (EXPORT lsdslam-targets DESTINATION lib/cmake/lsdslam NAMESPACE ::)

install(DIRECTORY src/ DESTINATION include/lsdslam
        FILES_MATCHING 
		PATTERN "*.h"
		PATTERN "*.hpp")

install(DIRECTORY thirdparty/Sophus DESTINATION include/lsdslam/
	FILES_MATCHING 
	PATTERN "*.h"
	PATTERN "*.hpp")

set(APPNAME lsdslam)
include(CMakePackageConfigHelpers) 
write_basic_package_version_file( 
  "${CMAKE_CURRENT_BINARY_DIR}/${APPNAME}/${APPNAME}-config-version.cmake" 
  VERSION 0.0.1 
  COMPATIBILITY AnyNewerVersion 
) 
configure_file(cmake/${APPNAME}-config.cmake.in 
  "${CMAKE_CURRENT_BINARY_DIR}/${APPNAME}/${APPNAME}-config.cmake" 
  @ONLY
) 


install( 
  FILES 
    "${CMAKE_CURRENT_BINARY_DIR}/${APPNAME}/${APPNAME}-config.cmake" 
    "${CMAKE_CURRENT_BINARY_DIR}/${APPNAME}/${APPNAME}-config-version.cmake" 
  DESTINATION 
    lib/cmake/${APPNAME} 
) 
	
#rosbuild_link_boost(lsdslam thread)


# build live ros node
#add_executable(live_slam src/main_live_odometry.cpp)
#target_link_libraries(live_slam lsdslam)


# build image node
#add_executable(dataset_slam src/main_on_images.cpp)
#target_link_libraries(dataset_slam lsdslam)

