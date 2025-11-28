cmake_minimum_required( VERSION 3.22 )

project( CloudCompareProjects LANGUAGES CXX )

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
if(WIN32 )
	set( CLOUDCOMPARE_DEST_FOLDER $<IF:$<CONFIG:Release>,CloudCompare,Cloudcompare_$<CONFIG>> )
	set( CCVIEWER_DEST_FOLDER $<IF:$<CONFIG:Release>,ccViewer,ccViewer_$<CONFIG>> )
elseif( APPLE )
	set( CLOUDCOMPARE_DEST_FOLDER CloudCompare )
	set( CCVIEWER_DEST_FOLDER ccViewer )
elseif( UNIX )
	set( CLOUDCOMPARE_DEST_FOLDER bin )
	set( CCVIEWER_DEST_FOLDER bin )
endif()

# Load advanced scripts
include( CMakeInclude )
include( Install )

# Add external libraries
include( CMakeExternalLibs )

if( WIN32 )
	# list that track shared lib installed via InstallShared function (see Install.cmake)
	# It allows to have a "robust" QT deployement
	set( CC_SHARED_LIB_FILENAMES "" CACHE INTERNAL "Shared lib tracking for Qt Deployement" FORCE)
	set( INSTALL_DESTINATIONS ${CLOUDCOMPARE_DEST_FOLDER} )

	if( ${OPTION_BUILD_CCVIEWER} )
		list( APPEND INSTALL_DESTINATIONS ${CCVIEWER_DEST_FOLDER} )
	endif()
elseif( APPLE )
	set( INSTALL_DESTINATIONS ${CLOUDCOMPARE_DEST_FOLDER}/${CLOUDCOMPARE_DEST_FOLDER}.app/Contents/Frameworks )

	if( ${OPTION_BUILD_CCVIEWER} )
		list( APPEND INSTALL_DESTINATIONS ${CCVIEWER_DEST_FOLDER}/${CCVIEWER_DEST_FOLDER}.app/Contents/Frameworks )
	endif()
elseif( UNIX )
	# RPATH Linux/Unix: (dynamic) libs are put in $prefix/$lib/cloudcompare,
	# since they are only used by qCC/ccViewer
	# GNUInstallDirs is already setup by the call to qt_standard_project_setup
	set( LINUX_INSTALL_SHARED_DESTINATION "${CMAKE_INSTALL_FULL_LIBDIR}/cloudcompare" )
	set( CMAKE_INSTALL_RPATH ${LINUX_INSTALL_SHARED_DESTINATION} )
	set( INSTALL_DESTINATIONS ${CMAKE_INSTALL_PREFIX})
endif()


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

# Clang format checks
include( Format )
