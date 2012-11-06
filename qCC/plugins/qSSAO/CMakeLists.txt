cmake_minimum_required(VERSION 2.8)

option( INSTALL_QSSAO_PLUGIN "Check to install qSSAO plugin" OFF )

if (INSTALL_QSSAO_PLUGIN)

#CloudCompare 'SSAO' plugin
project( QSSAO_PLUGIN_DLL )

add_subdirectory (Randomkit)

set( CC_SHADER_FOLDER SSAO )
include( ../CMakePluginTpl.cmake )

target_link_libraries( ${PROJECT_NAME} RandomKit )
include_directories( ${RANDOMKIT_INCLUDE_DIR} )

endif()
