option( PLUGIN_STANDARD_QFACETS "Check to install qFACETS plugin" OFF )

# CloudCompare 'FACETS' plugin
if (PLUGIN_STANDARD_QFACETS)
	if (NOT OPTION_USE_SHAPE_LIB)
		message( SEND_ERROR "ShapeLib is required to compile this plugin (enable OPTION_USE_SHAPE_LIB)" )
	endif()

	project( QFACETS_PLUGIN )

    AddPlugin( NAME ${PROJECT_NAME} )

	#we need Color Scale Manager and Editor importer
	file( GLOB CC_PLUGIN_CUSTOM_HEADER_LIST ${CloudCompare_SOURCE_DIR}/ccColorScale*.h )
	file( GLOB CC_PLUGIN_CUSTOM_SOURCE_LIST ${CloudCompare_SOURCE_DIR}/ccColorScale*.cpp )
	set( CC_PLUGIN_CUSTOM_UI_LIST ${CloudCompare_SOURCE_DIR}/ui_templates/colorScaleEditorDlg.ui )
	
	include_directories( src )
	
	add_subdirectory( src )

	add_subdirectory( ui )
	
    include( CMakePluginTemplate )
    
    set_target_properties( ${PROJECT_NAME} PROPERTIES
        # FIXME Remove after everything has moved to targets
        # This is an exception to find ccColorScaleEditorDlg which means we have an odd dependency
        AUTOUIC_SEARCH_PATHS "${CloudCompare_SOURCE_DIR}/ui_templates;${CMAKE_CURRENT_SOURCE_DIR}/ui"
    )
    
    # FIXME These are required becaue we have dependencies outside of the api
    #   (at least ccColorScale* and something in "common")
    # We need to determine where those files should live to avoid this.
    target_include_directories( ${PROJECT_NAME}
        PRIVATE
            ${CloudCompare_SOURCE_DIR}
            ${CloudCompare_SOURCE_DIR}/../common
    )

	target_link_libraries( ${PROJECT_NAME}
        shapelib
    )
endif()
