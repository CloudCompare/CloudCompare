cmake_minimum_required( VERSION 3.10 )

project( CloudCompareProjects )

# One shouldn't generate the BUILD project directly in the SOURCES folder!
if ( ${CMAKE_CURRENT_BINARY_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR} )
	if ( NOT SAME_BUILD_AND_SOURCE_FOLDER_WARNING_ALREADY_ISSUED )
		message(FATAL_ERROR "It is not advised to BUILD the binaries directly in the SOURCE folder!\n If you want to proceed with this option, just CONFIGURE the project once again" )
		set( SAME_BUILD_AND_SOURCE_FOLDER_WARNING_ALREADY_ISSUED TRUE )
	endif()
endif()

# Add our cmake module path so we don't need relative paths for these
list( APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/" )

include( CMakePolicies )
include( CMakeSetCompilerOptions )
include( DeployQt )

# CCViewer
option( OPTION_BUILD_CCVIEWER "Check to compile CCViewer project" ON )

# Testing
option( BUILD_TESTING "Build tests for CC" OFF )
if ( BUILD_TESTING )
	include( CTest )
endif()

# Default debug suffix for libraries.
set( CMAKE_DEBUG_POSTFIX "d" )

# Define target folders
# (now that ccViewer can have its own plugins, qCC and ccViewer must fall in separate folders!
if(WIN32 OR APPLE)
	set( CLOUDCOMPARE_DEST_FOLDER CloudCompare )
	set( CCVIEWER_DEST_FOLDER ccViewer )
else()
	set( CLOUDCOMPARE_DEST_FOLDER bin )
	set( CCVIEWER_DEST_FOLDER bin )
endif()

if( WIN32 )
    set( INSTALL_DESTINATIONS ${CLOUDCOMPARE_DEST_FOLDER} )

    if( ${OPTION_BUILD_CCVIEWER} )
        list( APPEND INSTALL_DESTINATIONS ${CCVIEWER_DEST_FOLDER} )
	endif()
elseif( UNIX AND NOT APPLE )
	# RPATH Linux/Unix: (dynamic) libs are put in $prefix/$lib/cloudcompare,
	# since they are only used by qCC/ccViewer
	include( GNUInstallDirs )
	set( LINUX_INSTALL_SHARED_DESTINATION "${CMAKE_INSTALL_FULL_LIBDIR}/cloudcompare" )
	set( CMAKE_INSTALL_RPATH ${LINUX_INSTALL_SHARED_DESTINATION} )
	set( INSTALL_DESTINATIONS ${CMAKE_INSTALL_PREFIX})
endif()

# Load advanced scripts
include( CMakeInclude )
include( Install )

# Add external libraries
include( CMakeExternalLibs )

# Internal libs used by both CloudCompare & ccViewer
add_subdirectory( libs )

# Plugins
add_subdirectory( plugins )

# qCC
add_subdirectory( qCC )

# CCViewer
if( OPTION_BUILD_CCVIEWER )
	add_subdirectory( ccViewer )
endif()
