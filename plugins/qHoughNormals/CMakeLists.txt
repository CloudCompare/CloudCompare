cmake_minimum_required(VERSION 3.0)

option( INSTALL_QHOUGH_NORMALS_PLUGIN "Check to install qHoughNormals plugin" OFF )

# CloudCompare 'Hough Normals' plugin (see https://github.com/aboulch/normals_Hough)
if (INSTALL_QHOUGH_NORMALS_PLUGIN)

	set( EIGEN_ROOT_DIR "" CACHE PATH "Eigen root directory" )
	if ( NOT EIGEN_ROOT_DIR )
		message( SEND_ERROR "No Eigen root directory specified (EIGEN_ROOT_DIR)" )
	endif()
	
	set( NANOFLANN_ROOT_DIR "" CACHE PATH "nanoflann root directory" )
	if ( NOT EIGEN_ROOT_DIR )
		message( SEND_ERROR "No nanoflann root directory specified (NANOFLANN_ROOT_DIR)" )
	endif()

    project( QHOUGH_NORMALS_PLUGIN )
    
    include( ../CMakePluginTpl.cmake )
    
    include_directories( ${CURRENT_SOURCE_DIR}/normals_Hough )
	include_directories( ${EIGEN_ROOT_DIR} )
	include_directories( ${NANOFLANN_ROOT_DIR}/include )
    
    target_link_libraries( ${PROJECT_NAME} )
	target_link_libraries( ${PROJECT_NAME} ${OPENGL_LIBRARIES} )
endif()
