cmake_minimum_required( VERSION 3.0 )

option( INSTALL_EXAMPLE_IO_PLUGIN "Check to install example I/O plugin" OFF )

if (INSTALL_EXAMPLE_IO_PLUGIN)
	project( ExampleIOPlugin )
	
	# Set to include this I/O plugin in ccViewer
	set( CC_IS_IO_PLUGIN 1 )

	include_directories( src )
	
	add_subdirectory( src )
	
	include( ../../CMakePluginTpl.cmake )    
endif()
