option( PLUGIN_STANDARD_QRANSAC_SD "Install qRansac_SD (Shape Detection) plugin" OFF )

if( PLUGIN_STANDARD_QRANSAC_SD )
	project( QRANSAC_SD_PLUGIN )

	AddPlugin( NAME ${PROJECT_NAME} )

	add_subdirectory( extern/RANSAC_SD )
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )

	# Causes the sub-clouds to be generated as partial clones of original cloud
	target_compile_definitions( ${PROJECT_NAME} PRIVATE POINTSWITHINDEX )
	
	target_link_libraries( ${PROJECT_NAME} QRANSAC_SD_PRIM_SHAPES_LIB )

	include( CMakePluginTemplate )
endif()
