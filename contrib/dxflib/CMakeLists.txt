cmake_minimum_required(VERSION 2.8)

# CC custom (external) CMake script for Ribbonsoft's DXF Lib
project( DXF_LIB )

file( GLOB header_list ${DXF_LIB_SRC_DIR}/*.h)
file( GLOB source_list ${DXF_LIB_SRC_DIR}/*.cpp)

add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} )

if (WIN32)
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS _CRT_SECURE_NO_DEPRECATE )
endif()
