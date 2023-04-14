option( PLUGIN_STANDARD_QTREEISO "Install qTreeIso plugin" OFF )

if( PLUGIN_STANDARD_QTREEISO )
	project( QTREEISO_PLUGIN )

	AddPlugin( NAME ${PROJECT_NAME} )
	
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )
endif()
