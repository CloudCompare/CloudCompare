cmake_minimum_required(VERSION 2.8)

option( INSTALL_QKINECT_PLUGIN "Check to install qKINECT plugin" OFF )

if (INSTALL_QKINECT_PLUGIN)

#CloudCompare 'Kinect' plugin
project( QKINECT_PLUGIN )

# libfreenect is needed to build library
include( LibfreenectSupport.cmake )

include( ../CMakePluginTpl.cmake )

target_link_libfreenect( ${PROJECT_NAME} ${CLOUDCOMPARE_DEST_FOLDER} )

endif()
