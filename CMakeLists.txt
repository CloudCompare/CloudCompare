cmake_minimum_required(VERSION 2.8)

project( CloudCompareProjects )

# One shouldn't generate the BUILD project directly in the SOURCES folder!
if ( ${CMAKE_CURRENT_BINARY_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR} )
	if ( NOT SAME_BUILD_AND_SOURCE_FOLDER_WARNING_ALREADY_ISSUED )
		message(FATAL_ERROR "It is not advised to BUILD the binaries directly in the SOURCE folder!\n If you want to proceed with this option, just CONFIGURE the project once again" )
		set( SAME_BUILD_AND_SOURCE_FOLDER_WARNING_ALREADY_ISSUED TRUE )
	endif()
endif()


# Default debug suffix for libraries
set( CMAKE_DEBUG_POSTFIX d )

if( UNIX )
        set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
        set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
endif()

if( MSVC )
	OPTION( OPTION_MP_BUILD "Check to activate multithreaded compilation with MSVC" OFF )
    if( ${OPTION_MP_BUILD} )
        set( CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS}\ /MP)
    endif()
endif()

# Default install folders
# (_debug suffix is automatically added for debug configurations)
set( INSTALL_DESTINATIONS CloudCompare )

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
add_subdirectory( libs/qCC_db ) # must always be included after CCFbo (dependency)

# liblas support
include( contrib/LiblasSupport.cmake )
# XIOT support
include( contrib/XIOTSupport.cmake )
# E57 support
include( contrib/E57Support.cmake )
# 3DXWARE (3dConnexion devices) support
include( contrib/3DXSupport.cmake )
# PDMS support
OPTION( OPTION_SUPPORT_MAC_PDMS_FORMAT "Build with .mac PDMS format" OFF )
# DXF support
include( contrib/DxfLibSupport.cmake )
# GDAL support
include( contrib/GDALSupport.cmake )
# FBX support
include( contrib/FBXSupport.cmake )

# qCC
add_subdirectory( qCC )

# CCViewer
if( ${OPTION_BUILD_CCVIEWER} )
	add_subdirectory( ccViewer )
endif()
