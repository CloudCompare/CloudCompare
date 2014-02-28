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

        # helper option for enabling c++11 support on linux
        option(BUILD_WITH_CPP11 "Build CloudCompare enabling c++11 support" OFF)
        if(BUILD_WITH_CPP11)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
        endif()

endif()

if( MSVC )
	OPTION( OPTION_MP_BUILD "Check to activate multithreaded compilation with MSVC" OFF )
    if( ${OPTION_MP_BUILD} )
        set( CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS}\ /MP)
    endif()
endif()

# Default install folders
# (on Windows the '_debug' suffix is automatically added for debug configurations)
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
add_subdirectory( libs/Glew )
add_subdirectory( libs/CCFbo )
add_subdirectory( libs/qCC_db ) # must always be included after CCFbo (dependency)
add_subdirectory( libs/qCC_io ) # must always be included after qCC_db (dependency)
add_subdirectory( libs/qCC_glWindow ) # must always be included after qCC_db (dependency)

# Define target folders
# (now that ccViewer can have its own plugins, qCC and ccViewer must fall in separate folders!
#  Therefore the target folder is the same for Windows and Unix/Linux)
set( CLOUDCOMPARE_DEST_FOLDER CloudCompare )
set( CCVIEWER_DEST_FOLDER ccViewer )
if (APPLE)
	set( CLOUDCOMPARE_MAC_BASE_DIR ${CMAKE_INSTALL_PREFIX}/${CLOUDCOMPARE_DEST_FOLDER}/CloudCompare.app  CACHE INTERNAL "CloudCompare bundle dir.")
	set( CCVIEWER_MAC_BASE_DIR ${CMAKE_INSTALL_PREFIX}/${CCVIEWER_DEST_FOLDER}/ccViewer.app CACHE INTERNAL "ccViewer bundle dir.")
endif()

#load plugins (potentially used by qCC and ccViewer below
add_subdirectory( plugins )

# qCC
add_subdirectory( qCC )

# CCViewer
if( ${OPTION_BUILD_CCVIEWER} )
	add_subdirectory( ccViewer )
endif()
