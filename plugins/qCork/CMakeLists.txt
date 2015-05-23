cmake_minimum_required(VERSION 2.8)

option( INSTALL_QCORK_PLUGIN "Check to install qCORK plugin" OFF )

if (INSTALL_QCORK_PLUGIN)

#CloudCompare Mesh CSG operations plugin (based on the Cork library)
project( QCORK_PLUGIN )

# Cork lib and MPIR are needed to build this plugin
include( CorkSupport.cmake )

include( ../CMakePluginTpl.cmake )

target_link_cork( ${PROJECT_NAME} )

endif()
