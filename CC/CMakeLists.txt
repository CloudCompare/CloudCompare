cmake_minimum_required(VERSION 2.8)

# Eliminate a warning when building in Windows that relates
# to static linking of Qt executables to qtmain.lib.
# This policy was introduced in CMake version 2.8.11.
# CMake version 2.8.11.2 warns when the policy is not set
# and uses OLD behavior.
if(POLICY CMP0020)
  cmake_policy(SET CMP0020 OLD)
endif()

# to compile CCLib only! (CMake implicitly imposes to declare a project before anything...)
project( CC_DUMMY_PROJECT )

# CC_CORE_LIB only relies on 'triangle' library...
add_subdirectory (triangle)

# Additional dependencies (only Qt in fact)
option( COMPILE_CC_CORE_LIB_WITH_QT "Check to compile CC_CORE_LIB with Qt (to allow parallel processing)" ON )
if (COMPILE_CC_CORE_LIB_WITH_QT)
	include( ../CMakeExternalLibs.cmake )
endif()

project( CC_CORE_LIB )

set( VERSION_MAJOR 1 )
set( VERSION_MINOR 0 )

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include )
if( MSVC )
	include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include/msvc )
endif()
include_directories( ${triangle_SOURCE_DIR} )

file( GLOB header_list include/*.h)
file( GLOB source_list src/*.cpp src/*.h)

add_library( ${PROJECT_NAME} SHARED ${header_list} ${source_list} )

target_link_libraries( ${PROJECT_NAME} triangle )

if ( COMPILE_CC_CORE_LIB_WITH_QT )
	if ( USE_QT5 )
		include_directories( ${Qt5Widgets_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS} )
		qt5_use_modules(${PROJECT_NAME} Core Widgets Concurrent)
	else()
		include_directories( ${QT_INCLUDE_DIR} )
		target_link_libraries( ${PROJECT_NAME} ${QT_LIBRARIES} )
	endif()
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS USE_QT )
endif()

# Load advanced scripts
include( ../CMakeInclude.cmake )

# Default preprocessors
set_default_cc_preproc( ${PROJECT_NAME} )

option( COMPILE_CC_CORE_LIB_SHARED "Check to compile CC_CORE_LIB as a shared library (DLL/so)" ON )
if ( COMPILE_CC_CORE_LIB_SHARED )
	if (WIN32)
		set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS CC_USE_AS_DLL )
	endif()

	# Install (shared) library to specified destinations
	if( UNIX )
		install_shared( CC_CORE_LIB lib 0 ) #default destination: /usr/lib
	else()
		foreach( dest ${INSTALL_DESTINATIONS} )
			install_shared( CC_CORE_LIB ${dest} 1 )
		endforeach()
	endif()
endif()