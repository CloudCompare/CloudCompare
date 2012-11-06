cmake_minimum_required(VERSION 2.8)

project( CloudComparePlugins )

# Auto add all subdirectories
file(GLOB subdirectories *)

foreach(dir ${subdirectories})
    if(IS_DIRECTORY ${dir})
		if (NOT ${dir} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR}/.svn ) #ignore svn subdir!
			message( STATUS "Auto-add plugin subdir:" ${dir})
			add_subdirectory (${dir})
		endif()
	endif()
endforeach()
