option( PLUGIN_STANDARD_QPOISSON_RECON "Install qPoissonRecon plugin" OFF )

if( PLUGIN_STANDARD_QPOISSON_RECON )
	project( QPOISSON_RECON_PLUGIN )
		
	AddPlugin( NAME ${PROJECT_NAME} )

	add_subdirectory( extern/PoissonRecon EXCLUDE_FROM_ALL )
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )

	include( CMakePluginTemplate )
	
	target_link_libraries( ${PROJECT_NAME} QPOISSON_RECON_LIB )
endif()
