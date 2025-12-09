option( PLUGIN_STANDARD_QPCV "Install qPCV plugin" OFF )

if( PLUGIN_STANDARD_QPCV )
	project( QPCV_PLUGIN )

	find_package( OpenGL MODULE REQUIRED )
	if( NOT OPENGL_GLU_FOUND )
		message( FATAL_ERROR "OpenGL required by PCV plugin" )
	endif()

	AddPlugin( NAME ${PROJECT_NAME} )

	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )

	target_link_libraries( ${PROJECT_NAME} OpenGL::GLU )
endif()
