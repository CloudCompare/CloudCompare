cmake_minimum_required(VERSION 3.0)

option( INSTALL_QPCV_PLUGIN "Check to install qPCV plugin" OFF )

# CloudCompare 'PCV' plugin
if (INSTALL_QPCV_PLUGIN)

    project( QPCV_PLUGIN )
    
    add_subdirectory (PCV)
    
    include( ../CMakePluginTpl.cmake )
    
    target_link_libraries( ${PROJECT_NAME} PCV_LIB )
    include_directories( ${PCV_LIB_SOURCE_DIR} )

endif()
