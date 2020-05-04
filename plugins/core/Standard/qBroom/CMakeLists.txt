option( PLUGIN_STANDARD_QBROOM "Check to install qBroom plugin" OFF )

if (PLUGIN_STANDARD_QBROOM)

	#CloudCompare 'Virtual broom' plugin
	project( QBROOM_PLUGIN )
	
    AddPlugin( NAME ${PROJECT_NAME} )

	include_directories( src )
	
	add_subdirectory( src )
	
    include( CMakePluginTemplate )
	
	include_directories( ${QBROOM_LIB_INCLUDE_DIR} )

endif()
