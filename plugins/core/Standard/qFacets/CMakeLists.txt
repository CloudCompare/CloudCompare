cmake_minimum_required(VERSION 3.0)

option( PLUGIN_STANDARD_QFACETS "Check to install qFACETS plugin" OFF )

# CloudCompare 'FACETS' plugin
if (PLUGIN_STANDARD_QFACETS)
	if (NOT SHAPELIB_SOURCE_DIR)
		message( SEND_ERROR "ShapeLib is required to compile this plugin (enable OPTION_USE_SHAPE_LIB)" )
	endif()

	project( QFACETS_PLUGIN )

	include( CMakePolicies NO_POLICY_SCOPE )

	# we need includes from the main CC source dir
	include_directories( ${CloudCompare_SOURCE_DIR} )

	#we need Color Scale Manager and Editor importer
	file( GLOB CC_PLUGIN_CUSTOM_HEADER_LIST ${CloudCompare_SOURCE_DIR}/ccColorScale*.h )
	file( GLOB CC_PLUGIN_CUSTOM_SOURCE_LIST ${CloudCompare_SOURCE_DIR}/ccColorScale*.cpp )
	set( CC_PLUGIN_CUSTOM_UI_LIST ${CloudCompare_SOURCE_DIR}/ui_templates/colorScaleEditorDlg.ui )
	
	include_directories( src )
	
	add_subdirectory( src )

	add_subdirectory( ui )
	
	include( ../../../CMakePluginTpl.cmake )

	#add IO support
	include_directories( ${QCC_IO_LIB_SOURCE_DIR} )
	target_link_libraries( ${PROJECT_NAME} QCC_IO_LIB )
	
	#add shapelib support
	target_link_libraries( ${PROJECT_NAME} SHAPELIB )
	include_directories( ${SHAPELIB_SOURCE_DIR} )
	target_compile_definitions( ${PROJECT_NAME} PRIVATE CC_SHP_SUPPORT )
endif()
