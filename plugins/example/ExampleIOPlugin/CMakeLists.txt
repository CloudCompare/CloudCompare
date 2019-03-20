cmake_minimum_required( VERSION 3.0 )

option( PLUGIN_EXAMPLE_IO "Check to install example I/O plugin" OFF )

if (PLUGIN_EXAMPLE_IO)
	project( ExampleIOPlugin )
	
	# Set to include this I/O plugin in ccViewer
	set( CC_IS_IO_PLUGIN 1 )

	include_directories( src )
	
	add_subdirectory( src )
	
	include( ../../CMakePluginTpl.cmake )    
endif()
