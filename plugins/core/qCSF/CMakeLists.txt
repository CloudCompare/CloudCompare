cmake_minimum_required(VERSION 3.0)

option( INSTALL_QCSF_PLUGIN "Check to install qCSF plugin" OFF )

if (INSTALL_QCSF_PLUGIN)

	#CloudCompare 'CSF' plugin
	project( QCSF_PLUGIN )

	include_directories( src )
	
	add_subdirectory( src )
	add_subdirectory( ui )
	
	include( ../../CMakePluginTpl.cmake )

endif()
