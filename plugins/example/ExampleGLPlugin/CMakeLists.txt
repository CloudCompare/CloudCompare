option( PLUGIN_EXAMPLE_GL "Install example GL plugin" OFF )

if ( PLUGIN_EXAMPLE_GL )
    project( ExampleGLPlugin )
    
    AddPlugin( NAME ${PROJECT_NAME} TYPE gl )
    
    include( CMakePluginTemplate )
    
    add_subdirectory( include )
    add_subdirectory( src )
endif()
