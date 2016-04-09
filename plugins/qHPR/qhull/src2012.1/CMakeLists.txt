cmake_minimum_required(VERSION 3.0)

project( qhull )

set( CMAKE_AUTOMOC OFF )

set( SAMPLE_PROJECT_VERSION_MAJOR 2012 )
set( SAMPLE_PROJECT_VERSION_MINOR 1 )

#define global variables for use by other projects
set ( QHULL_LIB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE )

include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )

set( header_list geom.h io.h libqhull.h mem.h merge.h poly.h random.h qhull_a.h qset.h stat.h user.h )
set( source_list rboxlib.c user.c global.c stat.c io.c geom2.c poly2.c merge.c libqhull.c geom.c poly.c qset.c mem.c usermem.c userprintf.c userprintf_rbox.c random.c )

add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} )

# Add preprocessor definitions
set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS _CRT_SECURE_NO_WARNINGS )
