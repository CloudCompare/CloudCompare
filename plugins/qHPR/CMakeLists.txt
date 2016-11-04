cmake_minimum_required(VERSION 3.0)

option( INSTALL_QHPR_PLUGIN "Check to install qHPR plugin" OFF )

# CloudCompare 'HPR' plugin
if (INSTALL_QHPR_PLUGIN)
    project( QHPR_PLUGIN )
    
    include( ../CMakePluginTpl.cmake )
    
    add_subdirectory (qhull/src2015.2)
    
    target_link_libraries( ${PROJECT_NAME} qhull )
    include_directories( ${QHULL_LIB_INCLUDE_DIR} )
endif()
