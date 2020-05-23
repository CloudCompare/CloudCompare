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

# CCViewer
option( OPTION_BUILD_CCVIEWER "Check to compile CCViewer project" ON )

# Quad buffer stereo support
option( OPTION_GL_QUAD_BUFFER_SUPPORT "Check to compile CloudCompare and ccViewer with Quad Buffer support" OFF )

if ( OPTION_GL_QUAD_BUFFER_SUPPORT )
    # Add the define for all libs and applications
	add_definitions( -DCC_GL_WINDOW_USE_QWINDOW )
endif()

# Testing
option( BUILD_TESTING "Build tests for CC" OFF )
if ( BUILD_TESTING )
	include( CTest )
endif()

# Default debug suffix for libraries.
set( CMAKE_DEBUG_POSTFIX "d" )

# Default install folders
# (on Windows - msvc generator - the '_debug' suffix is automatically added for debug configurations)
set( INSTALL_DESTINATIONS CloudCompare )

# RPATH Linux/Unix: (dynamic) libs are put in a subdir of prefix/lib,
# since they are only used by qCC/ccViewer
if( UNIX AND NOT APPLE )
	include( GNUInstallDirs )
	set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_FULL_LIBDIR}/cloudcompare")
endif()

# Define target folders
# (now that ccViewer can have its own plugins, qCC and ccViewer must fall in separate folders!
if(WIN32 OR APPLE) 
	set( CLOUDCOMPARE_DEST_FOLDER CloudCompare )
	set( CCVIEWER_DEST_FOLDER ccViewer )
	if (OPTION_GL_QUAD_BUFFER_SUPPORT)
		set ( CLOUDCOMPARE_DEST_FOLDER ${CLOUDCOMPARE_DEST_FOLDER}Stereo )
		set ( CCVIEWER_DEST_FOLDER ${CCVIEWER_DEST_FOLDER}Stereo )
	endif()
else()
	set( CLOUDCOMPARE_DEST_FOLDER bin )
	set( CCVIEWER_DEST_FOLDER bin )
endif()

# set default install folders
if( WIN32 )
    # (on Windows - msvc generator - the '_debug' suffix is automatically added for debug configurations)
    set( INSTALL_DESTINATIONS ${CLOUDCOMPARE_DEST_FOLDER} )
    
    if( ${OPTION_BUILD_CCVIEWER} )
        list( APPEND INSTALL_DESTINATIONS ${CCVIEWER_DEST_FOLDER} )
    endif()
elseif( APPLE )
	set( INSTALL_DESTINATIONS "${CLOUDCOMPARE_MAC_FRAMEWORK_DIR}" )
    
    if( ${OPTION_BUILD_CCVIEWER} )
        list( APPEND INSTALL_DESTINATIONS "${CCVIEWER_MAC_FRAMEWORK_DIR}" )
    endif()
elseif( UNIX )
	set( INSTALL_DESTINATIONS ${CMAKE_INSTALL_PREFIX}/bin )
endif()

# Load advanced scripts
include( CMakeInclude )
include( Install )

# Add external libraries
include( CMakeExternalLibs )

# Contrib. libraries (mainly for I/O)
include( AllSupport )

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
