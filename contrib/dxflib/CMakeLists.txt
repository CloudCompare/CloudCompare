cmake_minimum_required(VERSION 2.8)

# Eliminate a warning when building in Windows that relates
# to static linking of Qt executables to qtmain.lib.
# This policy was introduced in CMake version 2.8.11.
# CMake version 2.8.11.2 warns when the policy is not set
# and uses OLD behavior.
if(POLICY CMP0020)
  cmake_policy(SET CMP0020 OLD)
endif()

# CC custom (external) CMake script for Ribbonsoft's DXF Lib
project( DXF_LIB )

file( GLOB header_list ${DXF_LIB_SRC_DIR}/*.h)
file( GLOB source_list ${DXF_LIB_SRC_DIR}/*.cpp)

add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} )

if (WIN32)
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS _CRT_SECURE_NO_DEPRECATE )
endif()
