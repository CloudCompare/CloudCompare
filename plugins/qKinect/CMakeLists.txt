cmake_minimum_required(VERSION 3.0)

option( INSTALL_QKINECT_PLUGIN "Check to install qKINECT plugin" OFF )

# CloudCompare 'Kinect' plugin
if (INSTALL_QKINECT_PLUGIN)
    project( QKINECT_PLUGIN )
    
    # libfreenect is needed to build library
    include( LibfreenectSupport.cmake )
    
    include( ../CMakePluginTpl.cmake )
    
    target_link_libfreenect( ${PROJECT_NAME} ${CLOUDCOMPARE_DEST_FOLDER} )
endif()
