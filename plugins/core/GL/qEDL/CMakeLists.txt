option( PLUGIN_GL_QEDL "Install qEDL plugin" OFF )

if( PLUGIN_GL_QEDL )
	project( QEDL_GL_PLUGIN )
	
	AddPlugin(
		NAME ${PROJECT_NAME}
		TYPE gl
		SHADER_FOLDER EDL
	)
	
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( shaders )
endif()
