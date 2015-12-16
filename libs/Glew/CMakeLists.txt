cmake_minimum_required(VERSION 2.8)

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include )

project( GLEW_LIB )

add_library( ${PROJECT_NAME} STATIC include/GL/glew.h src/glew.c )

# Add custom prepocessor definitions
set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS GLEW_STATIC )
