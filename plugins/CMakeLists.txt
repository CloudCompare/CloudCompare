cmake_minimum_required(VERSION 2.8)

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

