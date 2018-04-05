cmake_minimum_required(VERSION 3.0)

option( INSTALL_QHPR_PLUGIN "Check to install qHPR plugin" OFF )

# CloudCompare 'HPR' plugin
if (INSTALL_QHPR_PLUGIN)
    project( QHPR_PLUGIN )
    	
	include_directories( src )
	
	add_subdirectory( src )
	add_subdirectory( src/qhull/src2015.2 )
	add_subdirectory( ui )
	
    include( ../../CMakePluginTpl.cmake )
    
    target_link_libraries( ${PROJECT_NAME} qhull )
    include_directories( ${QHULL_LIB_INCLUDE_DIR} )
endif()
