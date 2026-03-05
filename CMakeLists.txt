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

function(list_sources_recursive output_var search_dir)
	set(options "")
	set(oneValueArgs "")
	set(multiValueArgs EXCLUDE_DIRS)

	cmake_parse_arguments(PARSE_ARGV 2
			"ARG"
			"${options}"
			"${oneValueArgs}"
			"${multiValueArgs}"
	)

	file( GLOB_RECURSE files "${search_dir}/*.cpp" "${search_dir}/*.h")

	foreach (exclude_dir ${ARG_EXCLUDE_DIRS})
			list(FILTER files EXCLUDE REGEX ".*/${search_dir}/${exclude_dir}/.*")
	endforeach ()

	set(${output_var} ${files}  PARENT_SCOPE)
endfunction()

function(append_sources_to_file file_path)
	list_sources_recursive(source_list ${ARGN})
	string(REPLACE ";" "\n" source_list "${source_list}")
	file(APPEND "${SOURCES_LIST_FILE}" "${source_list}\n")
endfunction()

# The file path where we dump the list of source files to be formatted
set(SOURCES_LIST_FILE "${CMAKE_BINARY_DIR}/sources_to_format.txt")
file(REMOVE "${SOURCES_LIST_FILE}")

append_sources_to_file(source_list qCC EXCLUDE_DIRS extern)
append_sources_to_file(source_list ccViewer EXCLUDE_DIRS extern)
append_sources_to_file(source_list libs EXCLUDE_DIRS CCAppCommon/QDarkStyleSheet qCC_db/extern qCC_io/extern)
append_sources_to_file(source_list plugins/core/GL EXCLUDE_DIRS qSSAO/extern)
append_sources_to_file(source_list plugins/core/IO EXCLUDE_DIRS qE57IO/extern qPhotoscanIO/extern MeshIO/extern)

find_program(CLANG_FORMAT NAMES clang-format)

add_custom_target(
	format COMMAND ${CMAKE_COMMAND} -E echo "Format with: $<IF:$<BOOL:${CLANG_FORMAT}>,${CLANG_FORMAT},clang-format>"
	COMMAND $<IF:$<BOOL:${CLANG_FORMAT}>,${CLANG_FORMAT},clang-format> --version
	COMMAND $<IF:$<BOOL:${CLANG_FORMAT}>,${CLANG_FORMAT},clang-format> -i "--files=${SOURCES_LIST_FILE}"
)

add_custom_target(
	check-format
	COMMAND ${CMAKE_COMMAND} -E echo "Checking format with: $<IF:$<BOOL:${CLANG_FORMAT}>,${CLANG_FORMAT},clang-format>"
	COMMAND $<IF:$<BOOL:${CLANG_FORMAT}>,${CLANG_FORMAT},clang-format> --version
	COMMAND $<IF:$<BOOL:${CLANG_FORMAT}>,${CLANG_FORMAT},clang-format> --Werror --dry-run "--files=${SOURCES_LIST_FILE}"
)
