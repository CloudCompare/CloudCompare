cmake_minimum_required(VERSION 3.0)

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include )

project( CC_FBO_LIB )

file( GLOB header_list include/*.h )
file( GLOB source_list src/*.cpp )

add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} )

target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::OpenGL Qt5::OpenGLExtensions )
