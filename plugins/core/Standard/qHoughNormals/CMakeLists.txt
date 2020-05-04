option( PLUGIN_STANDARD_QHOUGH_NORMALS "Check to install qHoughNormals plugin" OFF )

# CloudCompare 'Hough Normals' plugin (see https://github.com/aboulch/normals_Hough)
if (PLUGIN_STANDARD_QHOUGH_NORMALS)

	set( EIGEN_ROOT_DIR "" CACHE PATH "Eigen root (contains the Eigen directory)" )
	if ( NOT EIGEN_ROOT_DIR )
		message( SEND_ERROR "No Eigen root directory specified (EIGEN_ROOT_DIR)" )
	endif()

	set( NANOFLANN_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/nanoflann" CACHE PATH "nanoflann root (contains the include directory)" )
	if ( NOT NANOFLANN_ROOT_DIR )
		message( SEND_ERROR "No nanoflann root directory specified (NANOFLANN_ROOT_DIR)" )
	endif()

	project( QHOUGH_NORMALS_PLUGIN )

    AddPlugin( NAME ${PROJECT_NAME} )

	include_directories( src )
	include_directories( src/normals_Hough )
	
	include_directories( ${NANOFLANN_ROOT_DIR}/include )
	include_directories( ${EIGEN_ROOT_DIR} )
	
	add_subdirectory( src )
	
    include( CMakePluginTemplate )
endif()
