option( PLUGIN_GL_QSSAO "Install qSSAO plugin" OFF )

if( PLUGIN_GL_QSSAO )
	
	project( QSSAO_GL_PLUGIN )
		
	AddPlugin(
		NAME ${PROJECT_NAME}
		TYPE gl
		SHADER_FOLDER SSAO
	)
	
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )
	add_subdirectory( shaders )

endif()
