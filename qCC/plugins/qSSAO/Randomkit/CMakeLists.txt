cmake_minimum_required(VERSION 2.8)

project( RandomKit )

set( SAMPLE_PROJECT_VERSION_MAJOR 1 )
set( SAMPLE_PROJECT_VERSION_MINOR 15 )

#define global variables for use by other projects
set ( RANDOMKIT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE )

include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )

add_library( ${PROJECT_NAME} STATIC erfc.h rk_isaac.c rk_mt.c rk_primitive.c rk_sobol.c )

# Add prepocessor definitions
set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS _CRT_SECURE_NO_WARNINGS )
