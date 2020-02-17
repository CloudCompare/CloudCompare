cmake_minimum_required(VERSION 3.0)

#Anytime we use COMPILE_DEFINITIONS_XXX we must define this policy!
#(and setting it outside of this file doesn't seem to work...)
#cmake_policy(SET CMP0043 OLD)

add_subdirectory( MiscLib )

include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/GfxTL )
include_directories( ${QRANSAC_SD_MISC_LIB_SOURCE_DIR} )

project( QRANSAC_SD_PRIM_SHAPES_LIB VERSION 1.1 )

file( GLOB header_list *.h GfxTL/*.h )
file( GLOB source_list *.cpp GfxTL/*.cpp )

add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} )

# Add preprocessor definitions
set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS _CRT_SECURE_NO_DEPRECATE _CRT_SECURE_NO_WARNINGS _SCL_SECURE_NO_WARNINGS TIMINGLEVEL0 _LIB )
set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS_RELEASE TIMINGLEVEL1)

if (OPENMP_FOUND AND NOT WIN32) #DGM: OpenMP doesn't work with Visual at least (the process loops infinitely)
	#set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS DOPARALLEL )
endif()
