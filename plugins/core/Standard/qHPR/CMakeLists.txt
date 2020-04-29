option( PLUGIN_STANDARD_QHPR "Check to install qHPR plugin" OFF )

# CloudCompare 'HPR' plugin
if (PLUGIN_STANDARD_QHPR)
    project( QHPR_PLUGIN )
    	
    AddPlugin( NAME ${PROJECT_NAME} )

	include_directories( src )
	
	add_subdirectory( src )
	add_subdirectory( src/qhull/src2015.2 )
	add_subdirectory( ui )
	
    include( CMakePluginTemplate )
    
    target_link_libraries( ${PROJECT_NAME} qhull )
    include_directories( ${QHULL_LIB_INCLUDE_DIR} )
endif()
