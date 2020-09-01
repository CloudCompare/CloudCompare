option( PLUGIN_STANDARD_QSRA "Install qSRA (Surface of Revolution Analysis) plugin" OFF )

# CloudCompare 'Surface of Revolution Analysis' plugin
if( PLUGIN_STANDARD_QSRA )
	# DXF support should be enabled for this plugin! (not mandatory though)
	if( NOT OPTION_USE_DXF_LIB )
		message( WARNING "qSRA plugin works best with dxflib support! (enable OPTION_USE_DXF_LIB)" )
	endif()
	
	project( QSRA_PLUGIN )

    AddPlugin( NAME ${PROJECT_NAME} )

	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )
endif()
