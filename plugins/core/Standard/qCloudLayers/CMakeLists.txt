option( PLUGIN_STANDARD_QCLOUDLAYERS "Install qCloudLayers release plugin" OFF )

if ( PLUGIN_STANDARD_QCLOUDLAYERS )
	project( QCLOUDLAYERS_PLUGIN )
	 
	AddPlugin( NAME ${PROJECT_NAME} )
		
	add_subdirectory( include )
	add_subdirectory( src )	
	add_subdirectory( ui )	
endif()
