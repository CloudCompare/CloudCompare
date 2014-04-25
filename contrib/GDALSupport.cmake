# ------------------------------------------------------------------------------
# GDAL support for CloudCompare
# ------------------------------------------------------------------------------

OPTION( OPTION_USE_GDAL "Build with GDAL support" OFF )
if( ${OPTION_USE_GDAL} )

	find_package(GDAL REQUIRED)
	
	if ( NOT GDAL_FOUND )
		message( SEND_ERROR "GDAL package not found!" )
	else()
		include_directories( ${GDAL_INCLUDE_DIR} )
	endif()

endif()

# Link project with GDAL library
function( target_link_GDAL ) # 2 arguments: ARGV0 = project name / ARGV1 = base lib export folder (optional)

if( ${OPTION_USE_GDAL} )
	
	if( GDAL_FOUND )

		target_link_libraries( ${ARGV0} ${GDAL_LIBRARY} )
		set_property( TARGET ${ARGV0} APPEND PROPERTY COMPILE_DEFINITIONS CC_GDAL_SUPPORT )
		
		if( WIN32 )
			
			#install DLLs
			if ( ARGV1 )
				file( GLOB dll_files ${GDAL_INCLUDE_DIR}/../bin/*.dll )
				foreach( gdalDLL ${dll_files} )
					if( NOT CMAKE_CONFIGURATION_TYPES )
						install( FILES ${gdalDLL} DESTINATION ${ARGV1} )
					else()
						install( FILES ${gdalDLL} CONFIGURATIONS Release DESTINATION ${ARGV1} )
						install( FILES ${gdalDLL} CONFIGURATIONS RelWithDebInfo DESTINATION ${ARGV1}_withDebInfo )
						install( FILES ${gdalDLL} CONFIGURATIONS Debug DESTINATION ${ARGV1}_debug )
					endif()
				endforeach()
			endif()
			
		endif()
	
	else()
	
		message( SEND_ERROR "GDAL package not found: can't link" )
	
	endif()

endif()

endfunction()
