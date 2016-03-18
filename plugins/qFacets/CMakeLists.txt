cmake_minimum_required(VERSION 3.0)

option( INSTALL_QFACETS_PLUGIN "Check to install qFACETS plugin" OFF )

# CloudCompare 'FACETS' plugin
if (INSTALL_QFACETS_PLUGIN)
	if (NOT SHAPELIB_SOURCE_DIR)
		message( SEND_ERROR "ShapeLib is required to compile this plugin (enable OPTION_USE_SHAPE_LIB)" )
	endif()

	project( QFACETS_PLUGIN_DLL )

	#As we are going to re-use several components of the main qCC project
	#we define its sources path relatively to the plugin's dir
	set(CloudCompare_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../qCC)
	include_directories( ${CloudCompare_SOURCE_DIR} )

	#we need Color Scale Manager and Editor importer
	file( GLOB CC_PLUGIN_CUSTOM_HEADER_LIST ${CloudCompare_SOURCE_DIR}/ccColorScale*.h )
	file( GLOB CC_PLUGIN_CUSTOM_SOURCE_LIST ${CloudCompare_SOURCE_DIR}/ccColorScale*.cpp )
	set( CC_PLUGIN_CUSTOM_UI_LIST ${CloudCompare_SOURCE_DIR}/ui_templates/colorScaleEditorDlg.ui )

	include( ../CMakePluginTpl.cmake )

	#add IO support
	include_directories( ${QCC_IO_LIB_SOURCE_DIR} )
	target_link_libraries( ${PROJECT_NAME} QCC_IO_LIB )
	
	#add shapelib support
	target_link_libraries( ${PROJECT_NAME} SHAPELIB )
	include_directories( ${SHAPELIB_SOURCE_DIR} )
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS CC_SHP_SUPPORT )
endif()
