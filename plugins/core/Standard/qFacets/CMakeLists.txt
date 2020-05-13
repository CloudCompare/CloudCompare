option( PLUGIN_STANDARD_QFACETS "Check to install qFACETS plugin" OFF )

if( PLUGIN_STANDARD_QFACETS )
	if( NOT OPTION_USE_SHAPE_LIB )
		message( FATAL_ERROR "ShapeLib is required to compile the Facets plugin (enable OPTION_USE_SHAPE_LIB)" )
	endif()

	project( QFACETS_PLUGIN )

	AddPlugin( NAME ${PROJECT_NAME} )
		
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )
		
	set_target_properties( ${PROJECT_NAME} PROPERTIES
		# FIXME Remove after ccColorScaleEditorDlg has moved
		# This is an exception to find ccColorScaleEditorDlg which means we have an odd dependency
		AUTOUIC_SEARCH_PATHS "${CloudCompare_SOURCE_DIR}/ui_templates;${CMAKE_CURRENT_SOURCE_DIR}/ui"
	)
	
	# FIXME These are required becaue we have dependencies outside of the API
	#	(at least ccColorScale* and ccQtHelpers in "common")
	# We need to determine where those files should live to avoid this.
	target_sources( ${PROJECT_NAME}
		PRIVATE
			${CloudCompare_SOURCE_DIR}/../common/ccQtHelpers.h
			# we need the Color Scale Manager and Editor importer
			${CloudCompare_SOURCE_DIR}/ccColorScaleEditorDlg.cpp
			${CloudCompare_SOURCE_DIR}/ccColorScaleEditorDlg.h
			${CloudCompare_SOURCE_DIR}/ccColorScaleEditorWidget.cpp
			${CloudCompare_SOURCE_DIR}/ccColorScaleEditorWidget.h
			${CloudCompare_SOURCE_DIR}/ccColorScaleSelector.cpp
			${CloudCompare_SOURCE_DIR}/ccColorScaleSelector.h
			${CloudCompare_SOURCE_DIR}/ui_templates/colorScaleEditorDlg.ui
	)
	target_include_directories( ${PROJECT_NAME}
		PRIVATE
			${CloudCompare_SOURCE_DIR}
			${CloudCompare_SOURCE_DIR}/../common
	)

	target_link_libraries( ${PROJECT_NAME}
		shapelib
	)
endif()
