cmake_minimum_required(VERSION 2.8)

project( SHAPELIB )

file( GLOB header_list *.h )
file( GLOB source_list *.c )

add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} )