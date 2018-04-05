cmake_minimum_required(VERSION 3.0)

option( INSTALL_QADDITIONAL_IO_PLUGIN "Check to install qAdditionalIO plugin" OFF )

if (INSTALL_QADDITIONAL_IO_PLUGIN)
	project( QADDITIONAL_IO_PLUGIN )

	set( CC_IS_IO_PLUGIN 1 )

	include_directories( src )

	add_subdirectory( src )
	add_subdirectory( ui )

	include( ../../CMakePluginTpl.cmake )
endif()
