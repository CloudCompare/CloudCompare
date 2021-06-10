option( PLUGIN_STANDARD_QCOMPASS "Check to install Compass plugin" OFF )

if( PLUGIN_STANDARD_QCOMPASS )
	project( QCOMPASS_PLUGIN )

	AddPlugin( NAME ${PROJECT_NAME} )

	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )
endif()
