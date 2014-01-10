cmake_minimum_required(VERSION 2.8)

option( INSTALL_QSRA_PLUGIN "Check to install qSRA (Surface of Revolution Analysis) plugin" OFF )

if (INSTALL_QSRA_PLUGIN)

	#DXF support should be enabled for this plugin! (not mandatory though)
	if(NOT OPTION_USE_DXF_LIB )
		message( WARNING "qSRA plugin works best with dxflib support! (enable OPTION_USE_DXF_LIB)" )
	endif()
	
	#CloudCompare 'Surface of Revolution Analysis' plugin
	project( QSRA_PLUGIN_DLL )

	#we need Color Scale Manager and Editor importer
	file( GLOB CC_PLUGIN_CUSTOM_HEADER_LIST ${CloudCompare_SOURCE_DIR}/ccColorScale*.h )
	file( GLOB CC_PLUGIN_CUSTOM_SOURCE_LIST ${CloudCompare_SOURCE_DIR}/ccColorScale*.cpp )
	#as well as the GL window class
	list( APPEND CC_PLUGIN_CUSTOM_HEADER_LIST ${CloudCompare_SOURCE_DIR}/ccGLWindow.h ${CloudCompare_SOURCE_DIR}/ccGuiParameters.h ${CloudCompare_SOURCE_DIR}/ccRenderingTools.h )
	list( APPEND CC_PLUGIN_CUSTOM_SOURCE_LIST ${CloudCompare_SOURCE_DIR}/ccGLWindow.cpp ${CloudCompare_SOURCE_DIR}/ccGuiParameters.cpp ${CloudCompare_SOURCE_DIR}/ccRenderingTools.cpp )
	#and the ccRenderToFileDlg class
	list( APPEND CC_PLUGIN_CUSTOM_HEADER_LIST ${CloudCompare_SOURCE_DIR}/ccRenderToFileDlg.h )
	list( APPEND CC_PLUGIN_CUSTOM_SOURCE_LIST ${CloudCompare_SOURCE_DIR}/ccRenderToFileDlg.cpp )

	include( ../CMakePluginTpl.cmake )
	
	#DXF support
	target_link_DXFLIB( ${PROJECT_NAME} )

endif()
