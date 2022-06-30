option( PLUGIN_STANDARD_QFACETS "Check to install qFACETS plugin" OFF )

if( PLUGIN_STANDARD_QFACETS )
	if( NOT OPTION_USE_SHAPE_LIB )
		message( FATAL_ERROR "ShapeLib is required to compile the Facets plugin (enable OPTION_USE_SHAPE_LIB)" )
	endif()

	project( QFACETS_PLUGIN )

	AddPlugin( NAME ${PROJECT_NAME} )
	target_compile_definitions( ${PROJECT_NAME} PRIVATE QT_NO_KEYWORDS )
		
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )
endif()
