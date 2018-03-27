cmake_minimum_required( VERSION 3.0 )

option( INSTALL_QEXAMPLE_IO_PLUGIN "Check to install qExampleIOPlugin plugin" OFF )

if (INSTALL_QEXAMPLE_IO_PLUGIN)
	project( QEXAMPLE_IO_PLUGIN )

	include_directories( src )
	
	add_subdirectory( src )
	
	include( ../CMakePluginTpl.cmake )    
endif()
