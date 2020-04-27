option( PLUGIN_GL_QEDL "Check to install qEDL plugin" OFF )

# CloudCompare 'EDL' plugin
if (PLUGIN_GL_QEDL)
    project( QEDL_GL_PLUGIN )
    
    AddPlugin( NAME ${PROJECT_NAME} TYPE gl )

	include_directories( src )
	
	add_subdirectory( src )
	
    set( CC_SHADER_FOLDER EDL )
    include( CMakePluginTemplate )
endif()
