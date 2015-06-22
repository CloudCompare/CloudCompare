cmake_minimum_required(VERSION 2.8)

option( INSTALL_QSSAO_PLUGIN "Check to install qSSAO plugin" OFF )

if (INSTALL_QSSAO_PLUGIN)

add_subdirectory (Randomkit)

#CloudCompare 'SSAO' plugin
project( QSSAO_PLUGIN )

set( CC_SHADER_FOLDER SSAO )
include( ../CMakePluginTpl.cmake )

target_link_libraries( ${PROJECT_NAME} QSSAO_RANDOMKIT_LIB )
include_directories( ${QSSAO_RANDOMKIT_LIB_SOURCE_DIR} )

endif()
