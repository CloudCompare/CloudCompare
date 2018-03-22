cmake_minimum_required(VERSION 3.0)

option( INSTALL_QADDITIONAL_IO_PLUGIN "Check to install qAdditionalIO plugin" OFF )

if (INSTALL_QADDITIONAL_IO_PLUGIN)
    project( INSTALL_QADDITIONAL_IO_PLUGIN )

	include_directories( src )
	
	add_subdirectory( src )
	add_subdirectory( ui )
	
    include( ../CMakePluginTpl.cmake )    
endif()
