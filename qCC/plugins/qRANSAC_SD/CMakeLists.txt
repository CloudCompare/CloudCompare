cmake_minimum_required(VERSION 2.8)

option( INSTALL_QRANSAC_SD_PLUGIN "Check to install qRansac_SD (Shape Detection) plugin" OFF )

if (INSTALL_QRANSAC_SD_PLUGIN)

#CloudCompare 'Ransac Shape Detection' plugin
project( QRANSAC_SD_PLUGIN_DLL )

add_subdirectory (RANSAC_SD_orig)

include( ../CMakePluginTpl.cmake )

target_link_libraries( ${PROJECT_NAME} PrimitiveShapes MiscLib )

include_directories( ${MiscLib_SOURCE_DIR} )
include_directories( ${PrimitiveShapes_SOURCE_DIR} )
include_directories( ${PrimitiveShapes_SOURCE_DIR}/GfxTL )

endif()
