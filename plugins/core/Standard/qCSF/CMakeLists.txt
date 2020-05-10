option( PLUGIN_STANDARD_QCSF "Install qCSF plugin" OFF )

if( PLUGIN_STANDARD_QCSF )
	project( QCSF_PLUGIN )

	AddPlugin( NAME ${PROJECT_NAME} )
	
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )
	
	include( CMakePluginTemplate )
endif()
