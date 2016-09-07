cmake_minimum_required(VERSION 3.0)

option( INSTALL_QHOUGH_NORMALS_PLUGIN "Check to install qHoughNormals plugin" OFF )

# CloudCompare 'Hough Normals' plugin (see https://github.com/aboulch/normals_Hough)
if (INSTALL_QHOUGH_NORMALS_PLUGIN)

    project( QHOUGH_NORMALS_PLUGIN )
    
    include( ../CMakePluginTpl.cmake )
    
    include_directories( ${CURRENT_SOURCE_DIR}/normals_Hough )
    
    target_link_libraries( ${PROJECT_NAME} PCV_LIB )
	target_link_libraries( ${PROJECT_NAME} ${OPENGL_LIBRARIES} )
endif()
