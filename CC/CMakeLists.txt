cmake_minimum_required(VERSION 3.0)

cmake_policy(PUSH)

# Options
option( COMPILE_CC_CORE_LIB_WITH_QT "Check to compile CC_CORE_LIB with Qt (to enable parallel processing)" ON )
option( COMPILE_CC_CORE_LIB_WITH_CGAL "Check to compile CC_CORE_LIB with CGAL lib. (to enable Delaunay 2.5D triangulation with a GPL compliant licence)" OFF )
option( COMPILE_CC_CORE_LIB_WITH_TBB " Check to compile CC_CORE_LIB with Intel Threading Building Blocks lib (enables some parallel processing )" OFF )
option( COMPILE_CC_CORE_LIB_SHARED "Check to compile CC_CORE_LIB as a shared library (DLL/so)" ON )

# to compile CCLib only! (CMake implicitly imposes to declare a project before anything...)
project( CC_CORE_LIB VERSION 1.0 )

include ( ../cmake/CMakePolicies.cmake )
include ( ../cmake/CMakeSetCompilerOptions.cmake )

# CGAL takes precedence over triangle
if (COMPILE_CC_CORE_LIB_WITH_CGAL)
	include( cmake/CGALSupport.cmake )
endif()

if (COMPILE_CC_CORE_LIB_WITH_TBB)
	include( cmake/FindTBB.cmake )
else()
	if (NOT WIN32)
		message( WARNING "CCLib configured without parallel algorithm capabilities - see COMPILE_CC_CORE_LIB_WITH_TBB" )
	endif()
endif()

# Additional dependencies (only Qt in fact)
if (COMPILE_CC_CORE_LIB_WITH_QT)
	include( ../cmake/CMakeExternalLibs.cmake )
endif()

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include )

file( GLOB header_list include/*.h)
file( GLOB source_list src/*.cpp src/*.h)

if (COMPILE_CC_CORE_LIB_SHARED)
	add_library( ${PROJECT_NAME} SHARED ${header_list} ${source_list} )
else()
	add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} )
endif()

if (COMPILE_CC_CORE_LIB_WITH_CGAL)
	target_link_libraries( ${PROJECT_NAME} ${CGAL_LIBRARIES} )
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS USE_CGAL_LIB )
endif()

if (COMPILE_CC_CORE_LIB_WITH_TBB)
	include_directories( ${TBB_INCLUDE_DIRS} )
	target_link_libraries( ${PROJECT_NAME} ${TBB_LIBRARIES} )
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS USE_TBB )
endif()

if ( COMPILE_CC_CORE_LIB_WITH_QT )
	include_directories( ${Qt5Widgets_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS} )
	target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Widgets Qt5::Concurrent)
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS USE_QT )
endif()

# Load advanced scripts
include( ../cmake/CMakeInclude.cmake )

if ( COMPILE_CC_CORE_LIB_SHARED )
	if (WIN32)
		set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS CC_USE_AS_DLL )
	endif()

	# Install (shared) library to specified destinations
	if( WIN32 OR APPLE )
		foreach( dest ${INSTALL_DESTINATIONS} )
			install_shared( CC_CORE_LIB ${dest} 1 )
		endforeach()
	else()
		install_shared( CC_CORE_LIB ${CMAKE_INSTALL_LIBDIR}/cloudcompare 0 ) #default destination: /usr/lib/cloudcompare
	endif()
endif()

if (WIN32)
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS _CRT_SECURE_NO_WARNINGS )
endif()

set( CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DCC_DEBUG" )
set( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DCC_DEBUG" )

cmake_policy(POP)
