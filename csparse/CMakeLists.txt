cmake_minimum_required(VERSION 3.0)

project(csparse)

add_library(csparse STATIC csparse.h csparse.c)
add_library(::csparse ALIAS csparse)
target_include_directories(csparse
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
		$<INSTALL_INTERFACE:include>
)

install(TARGETS csparse EXPORT csparse-targets
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(EXPORT csparse-targets DESTINATION lib/cmake/csparse NAMESPACE ::)

install (FILES csparse.h DESTINATION include/)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/csparse-config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/csparse/csparse-config.cmake" 
  @ONLY
) 


install( 
  FILES 
    "${CMAKE_CURRENT_BINARY_DIR}/csparse/csparse-config.cmake" 
  DESTINATION 
    lib/cmake/csparse
) 
