cmake_minimum_required(VERSION 2.8)

option( INSTALL_QEDL_PLUGIN "Check to install qEDL plugin" OFF )

if (INSTALL_QEDL_PLUGIN)

#CloudCompare 'EDL' plugin
project( QEDL_PLUGIN )

set( CC_SHADER_FOLDER EDL )
include( ../CMakePluginTpl.cmake )

endif()
