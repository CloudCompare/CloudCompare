cmake_minimum_required(VERSION 3.0)

# CC custom (external) CMake script for Ribbonsoft's DXF Lib
project( DXF_LIB )

set( CMAKE_AUTOMOC OFF )

file( GLOB header_list src/*.h)
file( GLOB source_list src/*.cpp)

add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} )

if (WIN32)
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS _CRT_SECURE_NO_DEPRECATE )
endif()
