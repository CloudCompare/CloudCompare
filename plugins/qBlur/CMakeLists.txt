cmake_minimum_required(VERSION 3.0)

option( INSTALL_QBLUR_PLUGIN "Check to install qBlur plugin" OFF )

if (INSTALL_QBLUR_PLUGIN)

#CloudCompare 'Blur' plugin
project( QBLUR_PLUGIN )

include( ../CMakePluginTpl.cmake )

endif()
