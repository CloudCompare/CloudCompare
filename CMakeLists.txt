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


# Formatting stuff:
# This part has to be here as it needs all targets listed below to be fully declared

# The list of CMake targets for which we want to format/check formatting
# Since CloudCompare's repo has some submodules for 3rd party dependencies
# we use this mechanism to explicitly state what should be formatted
set(TARGETS_TO_FORMAT
	CCAppCommon
	CC_FBO_LIB
	CC_FBO_LIB
	CCPluginAPI
	CCPluginStub

	QCORE_IO_PLUGIN
	QLAS_IO_PLUGIN
	QADDITIONAL_IO_PLUGIN
)

# The file path where we dump the list of source files to be formatted
set(SOURCES_LIST_FILE "${CMAKE_BINARY_DIR}/sources_to_format.txt")
file(REMOVE "${SOURCES_LIST_FILE}")
foreach (TARGET ${TARGETS_TO_FORMAT})
	if (NOT TARGET ${TARGET})
		message(VERBOSE "${TARGET} does no exists, cannot be formatted")
		continue()
	endif ()

	message(VERBOSE "${TARGET} sources added to format target")

	get_target_property(TARGET_SOURCES ${TARGET} SOURCES)
	list(FILTER TARGET_SOURCES INCLUDE REGEX ".*\.(cpp|hpp|cu|c|h)$")
	# For some reasons this exclude is needed otherwise we do get .qrc files in the list
	list(FILTER TARGET_SOURCES EXCLUDE REGEX ".*\.(qrc)$")
	# Files in the build dir are generated so don't bother formatting them
	list(FILTER TARGET_SOURCES EXCLUDE REGEX "^${CMAKE_BINARY_DIR}/.*")
	string(REPLACE ";" "\n" TARGET_SOURCES2 "${TARGET_SOURCES}")
	file(APPEND "${SOURCES_LIST_FILE}" ${TARGET_SOURCES2})
	file(APPEND "${SOURCES_LIST_FILE}" "\n")
endforeach ()

add_custom_target(
	format
	COMMAND clang-format --version
	COMMAND clang-format -i "--files=${SOURCES_LIST_FILE}"
)

add_custom_target(
	check-format
	COMMAND clang-format --version
	COMMAND clang-format --Werror --dry-run "--files=${SOURCES_LIST_FILE}"
)
