option( PLUGIN_STANDARD_QCANUPO "Check to install qCanupo plugin" OFF )

if( PLUGIN_STANDARD_QCANUPO )
	project( QCANUPO_PLUGIN )
	
	AddPlugin( NAME ${PROJECT_NAME} )

	unset( DLIB_ROOT CACHE ) #deprecated
	set( DLIB_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/contrib/dlib CACHE PATH "dlib library root directory" )
	if ( NOT DLIB_ROOT_DIR )
		message( FATAL_ERROR "No dlib library root specified (DLIB_ROOT_DIR)" )
	else()
		include_directories( ${DLIB_ROOT_DIR} )
	endif()
	
	option( QCANUPO_COMPILE_PRIVATE_METHODS "Compile qCanupo plugin with private methods (CNRS/UEB only)" OFF )
	if( QCANUPO_COMPILE_PRIVATE_METHODS )
		target_compile_definitions( ${PROJECT_NAME} PRIVATE COMPILE_PRIVATE_CANUPO )
	endif()
	
	target_compile_definitions( ${PROJECT_NAME} PUBLIC DLIB_NO_GUI_SUPPORT)
	
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )
	
	# Don't forget the DLIB 'source.cpp' file
	target_sources( ${PROJECT_NAME}
		PRIVATE
		${DLIB_ROOT_DIR}/dlib/all/source.cpp
	)
endif()
