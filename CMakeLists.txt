cmake_minimum_required(VERSION 3.0)

cmake_policy(PUSH)

include ( CMakePolicies.cmake )
include( CMakeSetCompilerOptions.cmake )

project( CloudCompareProjects CXX)

# Default debug suffix for libraries.
set( CMAKE_DEBUG_POSTFIX "d" )

# One shouldn't generate the BUILD project directly in the SOURCES folder!
if ( ${CMAKE_CURRENT_BINARY_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR} )
	if ( NOT SAME_BUILD_AND_SOURCE_FOLDER_WARNING_ALREADY_ISSUED )
		message(FATAL_ERROR "It is not advised to BUILD the binaries directly in the SOURCE folder!\n If you want to proceed with this option, just CONFIGURE the project once again" )
		set( SAME_BUILD_AND_SOURCE_FOLDER_WARNING_ALREADY_ISSUED TRUE )
	endif()
endif()

if (WIN32) # this option is for win only. somebody should see if it makes sense to enable it on APPLE too.
    option(INSTALL_SYSTEM_LIBS "When installing needed system libs will be copied too" ON)
endif()

# Default install folders
# (on Windows - msvc generator - the '_debug' suffix is automatically added for debug configurations)
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

# Contrib. libraries (mainly for I/O)
include( contrib/AllSupport.cmake )

# Libs requested by both qCC & ccViewer
add_subdirectory( libs/CCFbo )
add_subdirectory( libs/qCC_db ) # must always be included after CCFbo (dependency)
add_subdirectory( libs/qCC_io ) # must always be included after qCC_db (dependency)
add_subdirectory( libs/qCC_glWindow ) # must always be included after qCC_db (dependency)
add_subdirectory( libs/qcustomplot )

# Define target folders
# (now that ccViewer can have its own plugins, qCC and ccViewer must fall in separate folders!
#  Therefore the target folder is the same for Windows and Unix/Linux)
set( CLOUDCOMPARE_DEST_FOLDER CloudCompare )
set( CCVIEWER_DEST_FOLDER ccViewer )
if (APPLE)
	set( CLOUDCOMPARE_MAC_BASE_DIR ${CMAKE_INSTALL_PREFIX}/${CLOUDCOMPARE_DEST_FOLDER}/CloudCompare.app  CACHE INTERNAL "CloudCompare bundle dir.")
	set( CCVIEWER_MAC_BASE_DIR ${CMAKE_INSTALL_PREFIX}/${CCVIEWER_DEST_FOLDER}/ccViewer.app CACHE INTERNAL "ccViewer bundle dir.")
endif()

#load plugins (potentially used by qCC and ccViewer)
add_subdirectory( plugins )

# qCC
add_subdirectory( qCC )

# CCViewer
if( ${OPTION_BUILD_CCVIEWER} )
	add_subdirectory( ccViewer )
endif()

cmake_policy(POP)
