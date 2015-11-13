cmake_minimum_required(VERSION 2.8)

cmake_policy(PUSH)
include ( CMakePolicies.cmake )

# Set a consistent MACOSX_RPATH default across all CMake versions.
# When CMake 2.8.12 is required, change this default to 1.
# When CMake 3.0.0 is required, remove this block (see CMP0042).
if ( APPLE )
    if ( NOT DEFINED CMAKE_MACOSX_RPATH )
        set( CMAKE_MACOSX_RPATH 0 )
    endif()
endif()

project( CloudCompareProjects )

# One shouldn't generate the BUILD project directly in the SOURCES folder!
if ( ${CMAKE_CURRENT_BINARY_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR} )
	if ( NOT SAME_BUILD_AND_SOURCE_FOLDER_WARNING_ALREADY_ISSUED )
		message(FATAL_ERROR "It is not advised to BUILD the binaries directly in the SOURCE folder!\n If you want to proceed with this option, just CONFIGURE the project once again" )
		set( SAME_BUILD_AND_SOURCE_FOLDER_WARNING_ALREADY_ISSUED TRUE )
	endif()
endif()


# Default debug suffix for libraries
if(MSVC)
  set( CMAKE_DEBUG_POSTFIX d )
endif()

# You need a c++11 Compiler to build CC
if( UNIX )
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
    if(COMPILER_SUPPORTS_CXX11)
        set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")
        set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
    else()
        message(ERROR "Your compiler does not support C++11")
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


# Experimental targets exporting from the build tree
# Note: target exporting from the install tree still need to be implemented
OPTION( OPTION_EXPORT_TARGETS "Check to export a CCPConfig.cmake and related targets from the build tree" OFF )
if( ${OPTION_EXPORT_TARGETS} )
    # NOTES
    # once CloudCompareProjects (CCP) will be exported from the build tree other people that whant to
    # use cloudcompare code - or e.g. write plugins they will be able to find the package in this way
    # from within their cmakelists:
        #-> find_package(CCP REQUIRED) # CloudCompareProjects
        #-> include_directories(${CCP_INCLUDE_DIRS})
    # and then they just need to link their plugins against ${CCP_LIBRARIES}

    set(TARGETS_TO_EXPORT CC_CORE_LIB CC_FBO_LIB GLEW_LIB QCC_DB_LIB QCC_GL_LIB QCC_IO_LIB triangle)
	
	# add our own contrib dependencies!
	if( ${OPTION_USE_DXF_LIB} )
		list( APPEND TARGETS_TO_EXPORT DXF_LIB )
	endif()

    if( ${INSTALL_QPCL_PLUGIN} )
        # we also export the shared lib produced by that
        set(TARGETS_TO_EXPORT ${TARGETS_TO_EXPORT} QPCL_PLUGIN QPCL_PLUGIN_UTILS_LIB)
    endif()

    if( ${OPTION_BUILD_CCVIEWER} )
        # we also export the shared lib produced by that
        set(TARGETS_TO_EXPORT ${TARGETS_TO_EXPORT} ccViewer)
    endif()

    export(PACKAGE CCP) # this will make CCP build-tree locable from anywhere on the system for the user
                        # how? read here: http://www.cmake.org/Wiki/CMake/Tutorials/Package_Registry

    # this exports targets from the build tree. See install(EXPORT) command to export targets from an installation tree.
    export(TARGETS ${TARGETS_TO_EXPORT} FILE "${PROJECT_BINARY_DIR}/CCPTargets.cmake")

    # configure the cmake.in file and place in the build tree so external apps may find it
    configure_file(CCPConfig.cmake.in "${PROJECT_BINARY_DIR}/CCPConfig.cmake" @ONLY)
endif()

cmake_policy(POP)
