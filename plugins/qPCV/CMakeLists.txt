cmake_minimum_required(VERSION 2.8)

option( INSTALL_QPCV_PLUGIN "Check to install qPCV plugin" OFF )

if (INSTALL_QPCV_PLUGIN)

#CloudCompare 'PCV' plugin
project( QPCV_PLUGIN )

add_subdirectory (PCV)

include( ../CMakePluginTpl.cmake )

target_link_libraries( ${PROJECT_NAME} PCV_LIB )
include_directories( ${PCV_LIB_SOURCE_DIR} )

endif()
