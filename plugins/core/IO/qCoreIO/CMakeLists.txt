cmake_minimum_required( VERSION 3.0 )

option( PLUGIN_IO_QCORE "Check to install the CoreIO plugin" ON )

if ( PLUGIN_IO_QCORE )
	project( QCORE_IO_PLUGIN )

	include( CMakePolicies NO_POLICY_SCOPE )

	set( CC_IS_IO_PLUGIN 1 )
	
	include_directories( src )
	include_directories( src/PDMS )

	add_subdirectory( src )
	add_subdirectory( ui )
	
	include( ../../../CMakePluginTpl.cmake )
endif()
