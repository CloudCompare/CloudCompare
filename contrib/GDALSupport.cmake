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
function( target_link_GDAL ) # 2 arguments: ARGV0 = project name / ARGV1 = shared lib export folder

if( ${OPTION_USE_GDAL} )
	
	if( GDAL_FOUND )

	target_link_libraries( ${ARGV0} ${GDAL_LIBRARY} )
		
		if( WIN32 )
			set_property( TARGET ${ARGV0} APPEND PROPERTY COMPILE_DEFINITIONS_RELEASE CC_GDAL_SUPPORT )
			set_property( TARGET ${ARGV0} APPEND PROPERTY COMPILE_DEFINITIONS_DEBUG CC_GDAL_SUPPORT )
		else()
			set_property( TARGET ${ARGV0} APPEND PROPERTY COMPILE_DEFINITIONS CC_GDAL_SUPPORT )
		endif()
	
	else()
	
		message( SEND_ERROR "GDAL package not found: can't link" )
	
	endif()

endif()

endfunction()
