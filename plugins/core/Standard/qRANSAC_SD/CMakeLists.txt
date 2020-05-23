option( PLUGIN_STANDARD_QRANSAC_SD "Install qRansac_SD (Shape Detection) plugin" OFF )

if( PLUGIN_STANDARD_QRANSAC_SD )
	project( QRANSAC_SD_PLUGIN )

	AddPlugin( NAME ${PROJECT_NAME} )

	add_subdirectory( extern/RANSAC_SD EXCLUDE_FROM_ALL )
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )
	
	target_link_libraries( ${PROJECT_NAME} QRANSAC_SD_PRIM_SHAPES_LIB )
endif()
