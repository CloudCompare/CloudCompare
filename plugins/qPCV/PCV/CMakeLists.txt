cmake_minimum_required(VERSION 2.8)

# Eliminate a warning when building in Windows that relates
# to static linking of Qt executables to qtmain.lib.
# This policy was introduced in CMake version 2.8.11.
# CMake version 2.8.11.2 warns when the policy is not set
# and uses OLD behavior.
if(POLICY CMP0020)
  cmake_policy(SET CMP0020 OLD)
endif()

include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CC_CORE_LIB_SOURCE_DIR}/include )
include_directories( ${EXTERNAL_LIBS_INCLUDE_DIR} )

project( PCV_LIB )

file( GLOB header_list *.h )
file( GLOB source_list *.cpp )

add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} )

target_link_libraries( ${PROJECT_NAME} CC_CORE_LIB )
target_link_libraries( ${PROJECT_NAME} ${EXTERNAL_LIBS_LIBRARIES} )

# Add prepocessor definitions
set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS NOMINMAX _CRT_SECURE_NO_WARNINGS )
if (WIN32)
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS CC_USE_AS_DLL )
endif()
