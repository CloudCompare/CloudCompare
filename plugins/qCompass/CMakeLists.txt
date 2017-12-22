cmake_minimum_required(VERSION 3.0)

option( INSTALL_QCOMPASS_PLUGIN "Check to install compass plugin" OFF )

# CloudCompare 'compass' plugin
if (INSTALL_QCOMPASS_PLUGIN)
	project( QCOMPASS_PLUGIN )

	#As we are going to re-use several components of the main qCC project
	#we define its sources path relatively to the plugin's dir
	set(CloudCompare_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../qCC)
	
	#include the whole CC directory (easier)
	include_directories( ${CloudCompare_SOURCE_DIR} )
	
	#we need ccOverlay classes
	file( GLOB CC_PLUGIN_CUSTOM_HEADER_LIST ${CloudCompare_SOURCE_DIR}/ccOverlayDialog*.h )
	file( GLOB CC_PLUGIN_CUSTOM_SOURCE_LIST ${CloudCompare_SOURCE_DIR}/ccOverlayDialog*.cpp )

	#we also need picking hub classes
	list( APPEND  CC_PLUGIN_CUSTOM_HEADER_LIST ${CloudCompare_SOURCE_DIR}/ccPickingListener.h )
	list( APPEND  CC_PLUGIN_CUSTOM_HEADER_LIST ${CloudCompare_SOURCE_DIR}/ccPickingHub.h )	
	list( APPEND  CC_PLUGIN_CUSTOM_SOURCE_LIST ${CloudCompare_SOURCE_DIR}/ccPickingHub.cpp )

	#not sure this is the correcrt way of doing this but adding in 
	#qCC build file so it knows where the ui_*.h files are

	#list( APPEND CC_PLUGIN_CUSTOM_UI_LIST ${CMAKE_CURRENT_SOURCE_DIR}/mapDlg.ui )
	#list( APPEND CC_PLUGIN_CUSTOM_UI_LIST ${CMAKE_CURRENT_SOURCE_DIR}/compassDlg.ui )

	set( CC_PLUGIN_CUSTOM_UI_LIST ${CMAKE_CURRENT_SOURCE_DIR}/mapDlg.ui )
	set( CC_PLUGIN_CUSTOM_UI_LIST ${CMAKE_CURRENT_SOURCE_DIR}/compassDlg.ui )

	include( ../CMakePluginTpl.cmake  )

endif()
