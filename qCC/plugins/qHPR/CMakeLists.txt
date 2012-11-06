cmake_minimum_required(VERSION 2.8)

option( INSTALL_QHPR_PLUGIN "Check to install qHPR plugin" OFF )

if (INSTALL_QHPR_PLUGIN)

#CloudCompare 'HPR' plugin
project( QHPR_PLUGIN_DLL )

add_subdirectory (qhull/src2012.1)

include( ../CMakePluginTpl.cmake )

target_link_libraries( ${PROJECT_NAME} qhull )
include_directories( ${QHULL_LIB_INCLUDE_DIR} )

endif()
