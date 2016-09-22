cmake_minimum_required(VERSION 2.8)

option( INSTALL_QCSF_PLUGIN "Check to install qCSF plugin" OFF )

if (INSTALL_QCSF_PLUGIN)

#CloudCompare 'CSF' plugin
project( QCSF_PLUGIN )

include( ../CMakePluginTpl.cmake )

endif()
