option( PLUGIN_STANDARD_QM3C2 "Check to install qM3C2 plugin" OFF )

if (PLUGIN_STANDARD_QM3C2)
	project( QM3C2_PLUGIN )

    AddPlugin( NAME ${PROJECT_NAME} )

	include_directories( src )

	add_subdirectory( src )

    include( CMakePluginTemplate )
endif()
