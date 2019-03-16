cmake_minimum_required(VERSION 3.0)

project( QSSAO_RANDOMKIT_LIB VERSION 1.15 )

set( CMAKE_AUTOMOC OFF )

include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )

add_library( ${PROJECT_NAME} STATIC erfc.h rk_isaac.c rk_mt.c rk_primitive.c rk_sobol.c )

# Add preprocessor definitions
set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS _CRT_SECURE_NO_WARNINGS )
