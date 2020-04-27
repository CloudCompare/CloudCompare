option( PLUGIN_STANDARD_QCSF "Check to install qCSF plugin" OFF )

if (PLUGIN_STANDARD_QCSF)

	#CloudCompare 'CSF' plugin
	project( QCSF_PLUGIN )

    AddPlugin( NAME ${PROJECT_NAME} )

	include_directories( src )
	
	add_subdirectory( src )
	add_subdirectory( ui )
	
    include( CMakePluginTemplate )

endif()
