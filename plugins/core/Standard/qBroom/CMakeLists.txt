option( PLUGIN_STANDARD_QBROOM "Install qBroom plugin" OFF )

if( PLUGIN_STANDARD_QBROOM )
	project( QBROOM_PLUGIN )
	
	AddPlugin( NAME ${PROJECT_NAME} )
	
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )
	
	include( CMakePluginTemplate )
endif()
