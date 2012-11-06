cmake_minimum_required(VERSION 2.8)

option( INSTALL_QKINECT_PLUGIN "Check to install qKINECT plugin" OFF )

if (INSTALL_QKINECT_PLUGIN)

#CloudCompare 'Kinect' plugin
project( QKINECT_PLUGIN_DLL )

# libfreenect is needed to build library
set( LIBFREENECT_INCLUDE_DIR "" CACHE PATH "Include directory of libfreenect" )
set( LIBFREENECT_LIBRARY_FILE "" CACHE FILE "Library file of libfreenect" )
if (WIN32)
	set( LIBFREENECT_SHARED_LIBRARY_FILE "" CACHE FILE "Shared library file of libfreenect (dll)" )
endif()

if ( NOT LIBFREENECT_INCLUDE_DIR )
    message( SEND_ERROR "Fill in the libfreenect include folder path (LIBFREENECT_INCLUDE_DIR)" )
endif()
if ( NOT LIBFREENECT_LIBRARY_FILE )
    message( SEND_ERROR "Fill in the libfreenect library file path (LIBFREENECT_LIBRARY_FILE)" )
endif()

include( ../CMakePluginTpl.cmake )

include_directories( ${LIBFREENECT_INCLUDE_DIR} )
include_directories( ${LIBFREENECT_INCLUDE_DIR}/libfreenect )
target_link_libraries( ${PROJECT_NAME} ${LIBFREENECT_LIBRARY_FILE} )

# post-build events: we need to export freenect.dll as well!
if (WIN32)
	if( LIBFREENECT_SHARED_LIBRARY_FILE )
		string( REPLACE \\ / LIBFREENECT_SHARED_LIBRARY_FILE ${LIBFREENECT_SHARED_LIBRARY_FILE} )
		install_ext( FILES ${LIBFREENECT_SHARED_LIBRARY_FILE} qCC qCC_debug )
	endif()
endif()

endif()
