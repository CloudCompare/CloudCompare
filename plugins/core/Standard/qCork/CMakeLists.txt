cmake_minimum_required(VERSION 3.0)

option( PLUGIN_STANDARD_QCORK "Check to install qCORK plugin" OFF )

# CloudCompare Mesh CSG operations plugin (based on the Cork library)
if (PLUGIN_STANDARD_QCORK)
    project( QCORK_PLUGIN )
    
    # Cork lib and MPIR are needed to build this plugin
    include( CorkSupport.cmake )
    
	include( ../../../CMakePluginTpl.cmake )
    
    target_link_cork( ${PROJECT_NAME} )
endif()
