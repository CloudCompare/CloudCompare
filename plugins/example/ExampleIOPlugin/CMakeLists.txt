option( PLUGIN_EXAMPLE_IO "Install example I/O plugin" OFF )

if (PLUGIN_EXAMPLE_IO)
	project( ExampleIOPlugin )

    AddPlugin( NAME ${PROJECT_NAME} TYPE io )
    
    include( CMakePluginTemplate )

    add_subdirectory( include )
    add_subdirectory( src )
endif()
