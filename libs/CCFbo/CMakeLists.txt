cmake_minimum_required(VERSION 3.0)

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include )

project( CC_FBO_LIB )

file( GLOB header_list include/*.h )
file( GLOB source_list src/*.cpp )

add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} )

qt5_use_modules(${PROJECT_NAME} Core OpenGL OpenGLExtensions )
