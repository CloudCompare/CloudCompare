option( PLUGIN_STANDARD_QM3C2 "Install qM3C2 plugin" OFF )

if( PLUGIN_STANDARD_QM3C2 )
	project( QM3C2_PLUGIN )

	AddPlugin( NAME ${PROJECT_NAME} )

	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )

	include( CMakePluginTemplate )
endif()
