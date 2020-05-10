option( PLUGIN_STANDARD_QHOUGH_NORMALS "Install qHoughNormals plugin" OFF )

# CloudCompare 'Hough Normals' plugin (see https://github.com/aboulch/normals_Hough)
if( PLUGIN_STANDARD_QHOUGH_NORMALS )
	project( QHOUGH_NORMALS_PLUGIN )

	AddPlugin( NAME ${PROJECT_NAME} )
	
	set( EIGEN_ROOT_DIR "" CACHE PATH "Eigen root (contains the Eigen directory)" )
	if ( NOT EIGEN_ROOT_DIR )
		message( SEND_ERROR "No Eigen root directory specified (EIGEN_ROOT_DIR)" )
	endif()

	target_include_directories( ${PROJECT_NAME} PRIVATE ${EIGEN_ROOT_DIR} )
	
	add_subdirectory( extern/nanoflann )
	add_subdirectory( extern/normals_Hough )
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )
		
	include( CMakePluginTemplate )
endif()
