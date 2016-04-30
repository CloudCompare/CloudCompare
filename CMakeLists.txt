cmake_minimum_required(VERSION 3.0)

cmake_policy(PUSH)

# set up rpaths before we do anything
if (APPLE)
	# see: https://cmake.org/Wiki/CMake_RPATH_handling
	set( CMAKE_SKIP_BUILD_RPATH  FALSE )
	set( CMAKE_BUILD_WITH_INSTALL_RPATH FALSE ) 
	set( CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE )
endif()

include ( CMakePolicies.cmake )

project( CloudCompareProjects )

include( CMakeSetCompilerOptions.cmake )

# Default debug suffix for libraries.
set( CMAKE_DEBUG_POSTFIX "d" )

# One shouldn't generate the BUILD project directly in the SOURCES folder!
if ( ${CMAKE_CURRENT_BINARY_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR} )
	if ( NOT SAME_BUILD_AND_SOURCE_FOLDER_WARNING_ALREADY_ISSUED )
		message(FATAL_ERROR "It is not advised to BUILD the binaries directly in the SOURCE folder!\n If you want to proceed with this option, just CONFIGURE the project once again" )
		set( SAME_BUILD_AND_SOURCE_FOLDER_WARNING_ALREADY_ISSUED TRUE )
	endif()
endif()

# Default install folders
# (on Windows - msvc generator - the '_debug' suffix is automatically added for debug configurations)
set( INSTALL_DESTINATIONS CloudCompare )

# RPATH Linux/Unix: (dynamic) libs are put in a subdir of prefix/lib,
# since they are only used by qCC/ccViewer
if( UNIX AND NOT APPLE )
	set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/cloudcompare")
endif()

# CCViewer
OPTION( OPTION_BUILD_CCVIEWER "Check to compile CCViewer project" ON )

# Quad buffer stereo support
OPTION( OPTION_GL_QUAD_BUFFER_SUPPORT "Check to compile CloudCompare and ccViewer with Quad Buffer support" OFF )

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

if (APPLE)
	set( CLOUDCOMPARE_MAC_BASE_DIR ${CMAKE_INSTALL_PREFIX}/${CLOUDCOMPARE_DEST_FOLDER}/CloudCompare.app  CACHE INTERNAL "CloudCompare bundle dir")
	set( CLOUDCOMPARE_MAC_FRAMEWORK_DIR ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Frameworks CACHE INTERNAL "CC framework dir" )
	set( CLOUDCOMPARE_MAC_PLUGIN_DIR ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/PlugIns/ccPlugins CACHE INTERNAL "CC plugin dir" )
	
	set( CCVIEWER_MAC_BASE_DIR ${CMAKE_INSTALL_PREFIX}/${CCVIEWER_DEST_FOLDER}/ccViewer.app CACHE INTERNAL "ccViewer bundle dir")
	set( CCVIEWER_MAC_FRAMEWORK_DIR ${CCVIEWER_MAC_BASE_DIR}/Contents/Frameworks CACHE INTERNAL "CCViewer framework dir" )
	set( CCVIEWER_MAC_PLUGIN_DIR ${CCVIEWER_MAC_BASE_DIR}/Contents/PlugIns/ccViewerPlugins CACHE INTERNAL "CCViewer plugin dir" )
endif()

# set default install folders
if( WIN32 )
    # (on Windows - msvc generator - the '_debug' suffix is automatically added for debug configurations)
    set( INSTALL_DESTINATIONS ${CLOUDCOMPARE_DEST_FOLDER} )
    
    if( ${OPTION_BUILD_CCVIEWER} )
        list( APPEND INSTALL_DESTINATIONS ${CCVIEWER_DEST_FOLDER} )
    endif()
else( APPLE )
	set( INSTALL_DESTINATIONS "${CLOUDCOMPARE_MAC_FRAMEWORK_DIR}" )
    
    if( ${OPTION_BUILD_CCVIEWER} )
        list( APPEND INSTALL_DESTINATIONS "${CCVIEWER_MAC_FRAMEWORK_DIR}" )
    endif()
else( UNIX )
  set( INSTALL_DESTINATIONS ${CMAKE_INSTALL_PREFIX}/bin )
endif()

# Load advanced scripts
include( CMakeInclude.cmake )

add_subdirectory( CC )

# Add external libraries
include( CMakeExternalLibs.cmake )

# Contrib. libraries (mainly for I/O)
include( contrib/AllSupport.cmake )

# Libs requested by both qCC & ccViewer
add_subdirectory( libs/CCFbo )
add_subdirectory( libs/qCC_db ) # must always be included after CCFbo (dependency)
add_subdirectory( libs/qCC_io ) # must always be included after qCC_db (dependency)
add_subdirectory( libs/qCC_glWindow ) # must always be included after qCC_db (dependency)
add_subdirectory( libs/qcustomplot )

#load plugins (potentially used by qCC and ccViewer)
add_subdirectory( plugins )

# qCC
add_subdirectory( qCC )

# CCViewer
if( ${OPTION_BUILD_CCVIEWER} )
	add_subdirectory( ccViewer )
endif()

cmake_policy(POP)
