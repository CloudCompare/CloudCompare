# Auto add all subdirectories
file( GLOB subdirectories * )

foreach( dir ${subdirectories} )
    if( IS_DIRECTORY ${dir} AND EXISTS ${dir}/CMakeLists.txt )
		# in case the user generates the BUILD project in the same folder as the sources!
		if ( NOT ${dir} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR}/CMakeFiles )
			message( STATUS "Auto-add 3rd party plugin: " ${dir} )
			add_subdirectory ( ${dir} )
		endif()
	endif()
endforeach()
