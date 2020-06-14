cmake_minimum_required(VERSION 3.0)

option( PLUGIN_STANDARD_QCSF "Check to install qCSF plugin" OFF )

if (PLUGIN_STANDARD_QCSF)

	#CloudCompare 'CSF' plugin
	project( QCSF_PLUGIN )

	include( CMakePolicies NO_POLICY_SCOPE )

	include_directories( src )
	
	add_subdirectory( src )
	add_subdirectory( ui )
	
	include( ../../../CMakePluginTpl.cmake )

endif()
