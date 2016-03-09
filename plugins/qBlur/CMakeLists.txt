cmake_minimum_required(VERSION 3.0)

option( INSTALL_QBLUR_PLUGIN "Check to install qBlur plugin" OFF )

# CloudCompare 'Blur' plugin
if (INSTALL_QBLUR_PLUGIN)
    project( QBLUR_PLUGIN )
    
    include( ../CMakePluginTpl.cmake )
endif()
