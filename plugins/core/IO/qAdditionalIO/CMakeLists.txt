option( PLUGIN_IO_QADDITIONAL "Check to install qAdditionalIO plugin" OFF )

if (PLUGIN_IO_QADDITIONAL)
	project( QADDITIONAL_IO_PLUGIN )

    AddPlugin( NAME ${PROJECT_NAME} TYPE io )

	include_directories( src )

	add_subdirectory( src )

    include( CMakePluginTemplate )
endif()
