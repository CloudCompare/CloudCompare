cmake_minimum_required(VERSION 3.0)

option( PLUGIN_STANDARD_QCOMPASS "Check to install compass plugin" OFF )

if (PLUGIN_STANDARD_QCOMPASS)

	# CloudCompare 'compass' plugin
	project( QCOMPASS_PLUGIN )

	include( CMakePolicies NO_POLICY_SCOPE )

	# we need includes from the main CC source dir
	include_directories( ${CloudCompare_SOURCE_DIR} )

	#we need ccOverlay classes
	file( GLOB CC_PLUGIN_CUSTOM_HEADER_LIST ${CloudCompare_SOURCE_DIR}/../common/ccOverlayDialog*.h )
	file( GLOB CC_PLUGIN_CUSTOM_SOURCE_LIST ${CloudCompare_SOURCE_DIR}/../common/ccOverlayDialog*.cpp )

	#we also need picking hub classes
	list( APPEND CC_PLUGIN_CUSTOM_HEADER_LIST ${CloudCompare_SOURCE_DIR}/../common/ccPickingListener.h )
	list( APPEND CC_PLUGIN_CUSTOM_HEADER_LIST ${CloudCompare_SOURCE_DIR}/../common/ccPickingHub.h )
	list( APPEND CC_PLUGIN_CUSTOM_SOURCE_LIST ${CloudCompare_SOURCE_DIR}/../common/ccPickingHub.cpp )

	include_directories( src )

	add_subdirectory( src )
	add_subdirectory( ui )

	include( ../../../CMakePluginTpl.cmake )
endif()
