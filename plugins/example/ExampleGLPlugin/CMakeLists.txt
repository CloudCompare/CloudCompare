cmake_minimum_required( VERSION 3.0 )

option( INSTALL_EXAMPLE_GL_PLUGIN "Check to install example GL plugin" OFF )

# CloudCompare example GL plugin
if ( INSTALL_EXAMPLE_GL_PLUGIN )
    project( ExampleGLPlugin )
    
    include( ../../CMakePluginTpl.cmake )
endif()
