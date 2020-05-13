option( PLUGIN_STANDARD_QHPR "Install qHPR plugin" OFF )

if( PLUGIN_STANDARD_QHPR )
	project( QHPR_PLUGIN )
		
	AddPlugin( NAME ${PROJECT_NAME} )
	
	add_subdirectory( extern/qhull EXCLUDE_FROM_ALL )
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )
		
	target_link_libraries( ${PROJECT_NAME} qhull )
	
	include( CMakePluginTemplate )	
endif()
