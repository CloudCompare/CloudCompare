cmake_minimum_required(VERSION 2.8)

# Eliminate a warning when building in Windows that relates
# to static linking of Qt executables to qtmain.lib.
# This policy was introduced in CMake version 2.8.11.
# CMake version 2.8.11.2 warns when the policy is not set
# and uses OLD behavior.
if(POLICY CMP0020)
  cmake_policy(SET CMP0020 OLD)
endif()

project( CloudComparePlugins )

# reset buffers (for MacOS bundle)
set( CLOUDCOMPARE_PLUGINS CACHE INTERNAL "CloudCompare plugin list" )
set( CCVIEWER_PLUGINS CACHE INTERNAL "ccViewer plugin list" )

# Auto add all subdirectories
file(GLOB subdirectories *)

foreach(dir ${subdirectories})
    if(IS_DIRECTORY ${dir})
		#in case the user generates the BUILD project in the same folder as the sources!
		if ( NOT ${dir} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR}/CMakeFiles )
			#if the user uses an old version of TortoiseSVN!
			if ( NOT ${dir} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR}/.svn )
				message( STATUS "Auto-add plugin subdir:" ${dir})
				add_subdirectory (${dir})
			endif()
		endif()
	endif()
endforeach()

