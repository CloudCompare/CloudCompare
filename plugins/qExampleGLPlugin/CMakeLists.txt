cmake_minimum_required( VERSION 3.0 )

option( INSTALL_QEXAMPLE_GL_PLUGIN "Check to install qExampleGLPlugin plugin" OFF )

# CloudCompare example GL plugin
if ( INSTALL_QEXAMPLE_GL_PLUGIN )
    project( QEXAMPLE_GL_PLUGIN )
    
    include( ../CMakePluginTpl.cmake )
endif()
