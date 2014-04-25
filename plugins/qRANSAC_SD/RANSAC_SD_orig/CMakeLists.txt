cmake_minimum_required(VERSION 2.8)

# Eliminate a warning when building in Windows that relates
# to static linking of Qt executables to qtmain.lib.
# This policy was introduced in CMake version 2.8.11.
# CMake version 2.8.11.2 warns when the policy is not set
# and uses OLD behavior.
if(POLICY CMP0020)
  cmake_policy(SET CMP0020 OLD)
endif()

add_subdirectory( MiscLib )

include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/GfxTL )
include_directories( ${QRANSAC_SD_MISC_LIB_SOURCE_DIR} )

project( QRANSAC_SD_PRIM_SHAPES_LIB )

set( SAMPLE_PROJECT_VERSION_MAJOR 1 )
set( SAMPLE_PROJECT_VERSION_MINOR 1 )

file( GLOB header_list *.h GfxTL/*.h )
file( GLOB source_list *.cpp GfxTL/*.cpp )

add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} )

# Add prepocessor definitions
set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS _CRT_SECURE_NO_DEPRECATE _CRT_SECURE_NO_WARNINGS _SCL_SECURE_NO_WARNINGS TIMINGLEVEL0 _LIB )
set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS_RELEASE TIMINGLEVEL1)
