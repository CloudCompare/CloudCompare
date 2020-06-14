cmake_minimum_required(VERSION 3.0)

option( PLUGIN_IO_QADDITIONAL "Check to install qAdditionalIO plugin" OFF )

if (PLUGIN_IO_QADDITIONAL)
	project( QADDITIONAL_IO_PLUGIN )

	include( CMakePolicies NO_POLICY_SCOPE )

	set( CC_IS_IO_PLUGIN 1 )

	include_directories( src )

	add_subdirectory( src )
	add_subdirectory( ui )

	include( ../../../CMakePluginTpl.cmake )
endif()
