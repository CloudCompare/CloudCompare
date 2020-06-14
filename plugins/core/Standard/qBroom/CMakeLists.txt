cmake_minimum_required(VERSION 3.0)

option( PLUGIN_STANDARD_QBROOM "Check to install qBroom plugin" OFF )

if (PLUGIN_STANDARD_QBROOM)

	#CloudCompare 'Virtual broom' plugin
	project( QBROOM_PLUGIN )
	
	include( CMakePolicies NO_POLICY_SCOPE )

	include_directories( src )
	
	add_subdirectory( src )
	add_subdirectory( ui )
	
	include( ../../../CMakePluginTpl.cmake )
	
	include_directories( ${QBROOM_LIB_INCLUDE_DIR} )

endif()
