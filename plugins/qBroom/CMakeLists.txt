cmake_minimum_required(VERSION 2.8)

option( INSTALL_QBROOM_PLUGIN "Check to install qBroom plugin" OFF )

if (INSTALL_QBROOM_PLUGIN)

#CloudCompare 'Virtual broom' plugin
project( QBROOM_PLUGIN )

include( ../CMakePluginTpl.cmake )

include_directories( ${QBROOM_LIB_INCLUDE_DIR} )

endif()
