cmake_minimum_required(VERSION 2.8)

# Eliminate a warning when building in Windows that relates
# to static linking of Qt executables to qtmain.lib.
# This policy was introduced in CMake version 2.8.11.
# CMake version 2.8.11.2 warns when the policy is not set
# and uses OLD behavior.
if(POLICY CMP0020)
  cmake_policy(SET CMP0020 OLD)
endif()

project( SHAPELIB )

file( GLOB header_list *.h )
file( GLOB source_list *.c )

add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} )

# Default preprocessors
set_default_cc_preproc( ${PROJECT_NAME} )
