cmake_minimum_required( VERSION 3.0 )

option( PLUGIN_EXAMPLE_GL "Check to install example GL plugin" OFF )

# CloudCompare example GL plugin
if ( PLUGIN_EXAMPLE_GL )
    project( ExampleGLPlugin )
    
    include( ../../CMakePluginTpl.cmake )
endif()
