cmake_minimum_required(VERSION 3.0)

option( INSTALL_QM3C2_PLUGIN "Check to install qM3C2 plugin" OFF )

if (INSTALL_QM3C2_PLUGIN)

	project( QM3C2_PLUGIN_DLL )

	include_directories( src )

	add_subdirectory( src )
	add_subdirectory( ui )

	include( ../../CMakePluginTpl.cmake )

endif()
