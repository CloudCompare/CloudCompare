option( PLUGIN_STANDARD_QFIT_SPLINE "Install qFitSpline plugin" OFF )

# CloudCompare 'Fit Spline' plugin
if( PLUGIN_STANDARD_QFIT_SPLINE )
	project( QFIT_SPLINE_PLUGIN )

	AddPlugin( NAME ${PROJECT_NAME} )
	
	set( EIGEN_ROOT_DIR "" CACHE PATH "Eigen root (contains the Eigen directory)" )
	if ( NOT EIGEN_ROOT_DIR )
		message( SEND_ERROR "No Eigen root directory specified (EIGEN_ROOT_DIR)" )
	endif()

	target_include_directories( ${PROJECT_NAME} PRIVATE ${EIGEN_ROOT_DIR} )
	
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )
endif()
