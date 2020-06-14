cmake_minimum_required(VERSION 3.0)

# CC custom (external) CMake script for Ribbonsoft's DXF Lib
project( DXF_LIB )

# no Qt stuff here
set( CMAKE_AUTOMOC OFF )
set( CMAKE_AUTOUIC OFF )
set( CMAKE_AUTORCC OFF )

file( GLOB header_list src/*.h)
file( GLOB source_list src/*.cpp)

add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} )

if (WIN32)
	target_compile_definitions( ${PROJECT_NAME} PRIVATE _CRT_SECURE_NO_DEPRECATE )
endif()
