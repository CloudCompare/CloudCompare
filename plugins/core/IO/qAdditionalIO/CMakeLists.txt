option( PLUGIN_IO_QADDITIONAL "Install qAdditionalIO plugin" OFF )

if( PLUGIN_IO_QADDITIONAL )
	project( QADDITIONAL_IO_PLUGIN )

    AddPlugin( NAME ${PROJECT_NAME} TYPE io )

    add_subdirectory( include )
    add_subdirectory( src )

    include( CMakePluginTemplate )
endif()
