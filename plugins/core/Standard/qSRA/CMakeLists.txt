option( PLUGIN_STANDARD_QSRA "Check to install qSRA (Surface of Revolution Analysis) plugin" OFF )

# CloudCompare 'Surface of Revolution Analysis' plugin
if (PLUGIN_STANDARD_QSRA)
	#DXF support should be enabled for this plugin! (not mandatory though)
	if(NOT OPTION_USE_DXF_LIB )
		message( WARNING "qSRA plugin works best with dxflib support! (enable OPTION_USE_DXF_LIB)" )
	endif()
	
	project( QSRA_PLUGIN )

    AddPlugin( NAME ${PROJECT_NAME} )

	#we need Color Scale Manager and Editor importer
	file( GLOB CC_PLUGIN_CUSTOM_HEADER_LIST ${CloudCompare_SOURCE_DIR}/ccColorScale*.h )
	file( GLOB CC_PLUGIN_CUSTOM_SOURCE_LIST ${CloudCompare_SOURCE_DIR}/ccColorScale*.cpp )
	#and the ccRenderToFileDlg class
	list( APPEND CC_PLUGIN_CUSTOM_HEADER_LIST ${CloudCompare_SOURCE_DIR}/ccRenderToFileDlg.h )
	list( APPEND CC_PLUGIN_CUSTOM_SOURCE_LIST ${CloudCompare_SOURCE_DIR}/ccRenderToFileDlg.cpp )

	#don't forget associated widgets(.ui)
	set( CC_PLUGIN_CUSTOM_UI_LIST	${CloudCompare_SOURCE_DIR}/ui_templates/colorScaleEditorDlg.ui
									${CloudCompare_SOURCE_DIR}/ui_templates/renderToFileDialog.ui )

    include( CMakePluginTemplate )
    
    set_target_properties( ${PROJECT_NAME} PROPERTIES
        # FIXME Remove after everything has moved to targets
        # This is an exception to find ccColorScaleEditorDlg which means we have an odd dependency
        AUTOUIC_SEARCH_PATHS "${CloudCompare_SOURCE_DIR}/ui_templates;${CMAKE_CURRENT_SOURCE_DIR}/ui"
    )

    # FIXME These are required becaue we have dependencies outside of the api
    #   (at least ccColorScale*, ccRenderToFileDlg, and something in "common")
    # We need to determine where those files should live to avoid this.
    target_include_directories( ${PROJECT_NAME}
        PRIVATE
            ${CloudCompare_SOURCE_DIR}
            ${CloudCompare_SOURCE_DIR}/../common
    )
	
	#DXF support
	target_link_DXFLIB( ${PROJECT_NAME} )
endif()
