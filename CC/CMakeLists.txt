cmake_minimum_required(VERSION 2.8)

cmake_policy(PUSH)
include ( ../CMakePolicies.cmake )

# Options
option( COMPILE_CC_CORE_LIB_WITH_QT "Check to compile CC_CORE_LIB with Qt (to enable parallel processing)" ON )
option( COMPILE_CC_CORE_LIB_WITH_TRIANGLE "Check to compile CC_CORE_LIB with Triangle lib. (to enable Delaunay 2.5D triangulation)" ON )
option( COMPILE_CC_CORE_LIB_WITH_CGAL "Check to compile CC_CORE_LIB with CGAL lib. (to enable Delaunay 2.5D triangulation with a GPL compliant licence)" OFF )
option( COMPILE_CC_CORE_LIB_SHARED "Check to compile CC_CORE_LIB as a shared library (DLL/so)" ON )

# to compile CCLib only! (CMake implicitly imposes to declare a project before anything...)
project( CC_DUMMY_PROJECT )

# CGAL takes precedence over triangle
if (COMPILE_CC_CORE_LIB_WITH_CGAL)
	SET(COMPILE_CC_CORE_LIB_WITH_TRIANGLE OFF)
	include (CGALSupport.cmake)
endif()

# else CC_CORE_LIB only relies on 'triangle' library...
if (COMPILE_CC_CORE_LIB_WITH_TRIANGLE)
	add_subdirectory (triangle)
endif()

# Additional dependencies (only Qt in fact)
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

if (COMPILE_CC_CORE_LIB_SHARED)
	add_library( ${PROJECT_NAME} SHARED ${header_list} ${source_list} )
else()
	add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} )
endif()


if (COMPILE_CC_CORE_LIB_WITH_TRIANGLE)
	target_link_libraries( ${PROJECT_NAME} triangle )
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS USE_TRIANGLE_LIB )
endif()


if (COMPILE_CC_CORE_LIB_WITH_CGAL)
	target_link_libraries( ${PROJECT_NAME} ${CGAL_LIBRARIES} )
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS USE_CGAL_LIB )
endif()

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

cmake_policy(POP)
