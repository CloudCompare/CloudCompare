option( PLUGIN_IO_QCORE "Install the CoreIO plugin" ON )

if( PLUGIN_IO_QCORE )
	project( QCORE_IO_PLUGIN )

    AddPlugin( NAME ${PROJECT_NAME} TYPE io )

    add_subdirectory( include )
    add_subdirectory( src )
	
    include( CMakePluginTemplate )
endif()
