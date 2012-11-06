cmake_minimum_required(VERSION 2.8)

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include )
include_directories( ${GLEW_SOURCE_DIR}/include )

project( CC_FBO )

file( GLOB header_list include/*.h )
file( GLOB source_list src/*.cpp )

add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} )

target_link_libraries( ${PROJECT_NAME} GLEW )

# Default preprocessors
set_default_cc_preproc( ${PROJECT_NAME} )

# Add custom prepocessor definitions
set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS USE_GLEW GLEW_STATIC )

