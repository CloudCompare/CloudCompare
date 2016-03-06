cmake_minimum_required(VERSION 3.0)

project( SHAPELIB )

set( CMAKE_AUTOMOC OFF )

file( GLOB header_list *.h )
file( GLOB source_list *.c )

add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} )