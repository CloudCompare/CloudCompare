cmake_minimum_required(VERSION 2.8)

option( INSTALL_QSRA_PLUGIN "Check to install qSRA (Surface of Revolution Analysis) plugin" OFF )

if (INSTALL_QSRA_PLUGIN)

	#DXF support should be enabled for this plugin! (not mandatory though)
	if(NOT OPTION_USE_DXF_LIB )
		message( WARNING "qSRA plugin works best with dxflib support! (enable OPTION_USE_DXF_LIB)" )
	endif()
	
	#CloudCompare 'Surface of Revolution Analysis' plugin
	project( QSRA_PLUGIN )
	
	#As we are going to re-use several components of the main qCC project
	#we define its sources path relatively to the plugin's dir
	set(CloudCompare_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../qCC)

	#include the whole CC directory (easier)
	include_directories( ${CloudCompare_SOURCE_DIR} )
	
	#we need Color Scale Manager and Editor importer
	file( GLOB CC_PLUGIN_CUSTOM_HEADER_LIST ${CloudCompare_SOURCE_DIR}/ccColorScale*.h )
	file( GLOB CC_PLUGIN_CUSTOM_SOURCE_LIST ${CloudCompare_SOURCE_DIR}/ccColorScale*.cpp )
	#and the ccRenderToFileDlg class
	list( APPEND CC_PLUGIN_CUSTOM_HEADER_LIST ${CloudCompare_SOURCE_DIR}/ccRenderToFileDlg.h )
	list( APPEND CC_PLUGIN_CUSTOM_SOURCE_LIST ${CloudCompare_SOURCE_DIR}/ccRenderToFileDlg.cpp )

	#don't forget associated widgets(.ui)
	set( CC_PLUGIN_CUSTOM_UI_LIST	${CloudCompare_SOURCE_DIR}/ui_templates/colorScaleEditorDlg.ui
									${CloudCompare_SOURCE_DIR}/ui_templates/renderToFileDialog.ui
									${CloudCompare_SOURCE_DIR}/ui_templates/displayOptionsDlg.ui )

	include( ../CMakePluginTpl.cmake )
	
	#DXF support
	target_link_DXFLIB( ${PROJECT_NAME} )

endif()
