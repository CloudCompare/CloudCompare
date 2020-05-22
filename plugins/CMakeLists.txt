project( CloudComparePlugins )

# Add our cmake module path so we don't need relative paths for these
list( APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/" )

include( Plugins )

set( CloudCompare_SOURCE_DIR "${CMAKE_SOURCE_DIR}/qCC" )
set( CloudComparePlugins_SOURCE_DIR "${CMAKE_SOURCE_DIR}/plugins" )

add_subdirectory( core )
add_subdirectory( example )

# Check for any private plugins and add them
get_filename_component( PRIVATE_DIR private ABSOLUTE )

if( EXISTS "${PRIVATE_DIR}" )	
	file( GLOB subdirectories "${PRIVATE_DIR}/*" )
	
	foreach( dir ${subdirectories} )
	    if( IS_DIRECTORY ${dir} AND EXISTS ${dir}/CMakeLists.txt )
			# In case the developer generated the BUILD project in the same folder as the sources
			if( NOT ${dir} STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}/CMakeFiles" )
				message( STATUS "Found private plugin: " ${dir} )
				add_subdirectory( ${dir} )
			endif()
		endif()
	endforeach()
endif()
