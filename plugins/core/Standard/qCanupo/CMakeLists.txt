option( PLUGIN_STANDARD_QCANUPO "Check to install qCanupo plugin" OFF )

if( PLUGIN_STANDARD_QCANUPO )
	project( QCANUPO_PLUGIN )

	AddPlugin( NAME ${PROJECT_NAME} )

	option( QCANUPO_COMPILE_PRIVATE_METHODS "Compile qCanupo plugin with private methods (CNRS/UEB only)" OFF )

	set( DLIB_NO_GUI_SUPPORT ON  CACHE BOOL "" FORCE )
	set( DLIB_USE_CUDA OFF  CACHE BOOL "" FORCE ) # we do not want to mess with CUDA
	add_subdirectory( contrib/dlib )

	if( QCANUPO_COMPILE_PRIVATE_METHODS )
		target_compile_definitions( ${PROJECT_NAME} PRIVATE COMPILE_PRIVATE_CANUPO )
	endif()

	target_link_libraries( ${PROJECT_NAME} dlib::dlib )

	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )

endif()
