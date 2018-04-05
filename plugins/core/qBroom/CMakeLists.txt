cmake_minimum_required(VERSION 3.0)

option( INSTALL_QBROOM_PLUGIN "Check to install qBroom plugin" OFF )

if (INSTALL_QBROOM_PLUGIN)

	#CloudCompare 'Virtual broom' plugin
	project( QBROOM_PLUGIN )
	
	include_directories( src )
	
	add_subdirectory( src )
	add_subdirectory( ui )
	
	include( ../../CMakePluginTpl.cmake )
	
	include_directories( ${QBROOM_LIB_INCLUDE_DIR} )

endif()
