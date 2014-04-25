cmake_minimum_required(VERSION 2.8)

# Eliminate a warning when building in Windows that relates
# to static linking of Qt executables to qtmain.lib.
# This policy was introduced in CMake version 2.8.11.
# CMake version 2.8.11.2 warns when the policy is not set
# and uses OLD behavior.
if(POLICY CMP0020)
  cmake_policy(SET CMP0020 OLD)
endif()

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include )
include_directories( ${GLEW_LIB_SOURCE_DIR}/include )

project( CC_FBO_LIB )

file( GLOB header_list include/*.h )
file( GLOB source_list src/*.cpp )

add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} )

target_link_libraries( ${PROJECT_NAME} GLEW_LIB )

# Default preprocessors
set_default_cc_preproc( ${PROJECT_NAME} )

# Add custom prepocessor definitions
set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS USE_GLEW GLEW_STATIC )

