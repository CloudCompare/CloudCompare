cmake_minimum_required(VERSION 3.0)

option( PLUGIN_STANDARD_QHPR "Check to install qHPR plugin" OFF )

# CloudCompare 'HPR' plugin
if (PLUGIN_STANDARD_QHPR)
    project( QHPR_PLUGIN )
    	
	include_directories( src )
	
	add_subdirectory( src )
	add_subdirectory( src/qhull/src2015.2 )
	add_subdirectory( ui )
	
	include( ../../../CMakePluginTpl.cmake )
    
    target_link_libraries( ${PROJECT_NAME} qhull )
    include_directories( ${QHULL_LIB_INCLUDE_DIR} )

	set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "plugins") 
	set_target_properties(qhull PROPERTIES FOLDER "3rdparty")
endif()
