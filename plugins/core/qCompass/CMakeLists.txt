cmake_minimum_required(VERSION 3.0)

option( INSTALL_QCOMPASS_PLUGIN "Check to install compass plugin" OFF )

# CloudCompare 'compass' plugin
if (INSTALL_QCOMPASS_PLUGIN)
	project( QCOMPASS_PLUGIN )

	# we need includes from the main CC source dir
	include_directories( ${CloudCompare_SOURCE_DIR} )

	#we need ccOverlay classes
	file( GLOB CC_PLUGIN_CUSTOM_HEADER_LIST ${CloudCompare_SOURCE_DIR}/ccOverlayDialog*.h )
	file( GLOB CC_PLUGIN_CUSTOM_SOURCE_LIST ${CloudCompare_SOURCE_DIR}/ccOverlayDialog*.cpp )

	#we also need picking hub classes
	list( APPEND  CC_PLUGIN_CUSTOM_HEADER_LIST ${CloudCompare_SOURCE_DIR}/ccPickingListener.h )
	list( APPEND  CC_PLUGIN_CUSTOM_HEADER_LIST ${CloudCompare_SOURCE_DIR}/ccPickingHub.h )
	list( APPEND  CC_PLUGIN_CUSTOM_SOURCE_LIST ${CloudCompare_SOURCE_DIR}/ccPickingHub.cpp )

	include_directories( src )

	add_subdirectory( src )
	add_subdirectory( ui )

	include( ../../CMakePluginTpl.cmake  )

endif()
