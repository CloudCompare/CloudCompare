cmake_minimum_required(VERSION 2.8)

project( CloudCompare )

# Default debug suffix for libraries
set( CMAKE_DEBUG_POSTFIX d )

if( UNIX )
	list( APPEND CMAKE_CXX_FLAGS "-fPIC" )
	list( APPEND CMAKE_C_FLAGS "-fPIC" )
endif()

if( MSVC )
	OPTION( OPTION_MP_BUILD "Check to activate multithreaded compilation with MSVC" OFF )
    if( ${OPTION_MP_BUILD} )
        set( CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS}\ /MP)
    endif()
endif()

# Default install folders
# (_debug suffix is automatically added for debug configurations)
set( INSTALL_DESTINATIONS qCC )

# CCViewer
OPTION( OPTION_BUILD_CCVIEWER "Check to compile CCViewer project" ON )
if( ${OPTION_BUILD_CCVIEWER} )
	list( APPEND INSTALL_DESTINATIONS ccViewer )
endif()

# Load advanced scripts
include( CMakeInclude.cmake )

add_subdirectory( CC )

# Add external libraries
include( CMakeExternalLibs.cmake )
# Libs requested by both qCC & ccViewer
add_subdirectory( libs/Glew )
add_subdirectory( libs/CCFbo )
add_subdirectory( libs/qCC_db )

# liblas support
include( LiblasSupport.cmake )
# XIOT support
include( XIOTSupport.cmake )
# E57 support
include( E57Support.cmake )
# PDMS support
OPTION( OPTION_SUPPORT_MAC_PDMS_FORMAT "Build with .mac PDMS format" OFF )

# qCC
add_subdirectory( qCC )

# CCViewer
if( ${OPTION_BUILD_CCVIEWER} )
	add_subdirectory( ccViewer )
endif()

