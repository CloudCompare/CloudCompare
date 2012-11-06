# ------------------------------------------------------------------------------
# Quick & dirty XIOT(X3D)+CMake support for CloudCompare
# ------------------------------------------------------------------------------

OPTION( OPTION_USE_XIOT "Build with XIOT (X3D support)" OFF )
if( ${OPTION_USE_XIOT} )

	# XIOT
	set( XIOT_INSTALL_DIR "" CACHE PATH "XIOT CMAKE INSTALL directory" )
	if ( NOT XIOT_INSTALL_DIR )
		message( SEND_ERROR "No XIOT install dir specified (XIOT_INSTALL_DIR)" )
	else()
		include_directories( ${XIOT_INSTALL_DIR}/include )
	endif()

endif()

# Export XIOT Dlls to specified destinations
function( target_link_XIOT ) # 3 arguments: ARGV0 = project name / ARGV1 = shared lib export folder (release) / ARGV2 = shared lib export folder (debug)
if( XIOT_INSTALL_DIR )
	
	string( REPLACE \\ / XIOT_LIBRARY_DIR ${XIOT_INSTALL_DIR}/lib )
	file( GLOB lib_files ${XIOT_LIBRARY_DIR}/*.* )
	target_link_libraries( ${ARGV0} ${lib_files} )
	
	if ( WIN32 )
		string( REPLACE \\ / XIOT_SHARED_LIBRARY_DIR ${XIOT_INSTALL_DIR}/bin )
		set( dll_files ${XIOT_SHARED_LIBRARY_DIR}/xiot.dll ${XIOT_SHARED_LIBRARY_DIR}/openFI.dll )
		foreach( dll_file ${dll_files} )
			if ( ARGV1 )
				install( FILES ${dll_file} CONFIGURATIONS Release DESTINATION ${ARGV1} )
			endif()
			if ( ARGV2 )
				install( FILES ${dll_file} CONFIGURATIONS Debug DESTINATION ${ARGV2} )
			endif()
		endforeach()
	endif()

	set_property( TARGET ${ARGV0} APPEND PROPERTY COMPILE_DEFINITIONS CC_X3D_SUPPORT )
endif()
endfunction()

