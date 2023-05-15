cmake_minimum_required( VERSION 3.10 )

option( PLUGIN_STANDARD_QTREEISO "Install qTreeIso plugin" OFF )

if( PLUGIN_STANDARD_QTREEISO )
	project( QTREEISO_PLUGIN )
	
	AddPlugin( NAME ${PROJECT_NAME} )	

	
	find_package(Boost 1.67 REQUIRED COMPONENTS graph)
	find_package(Eigen3 QUIET)
	
	
	
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )

	if(NOT Boost_FOUND)
		set( BOOST_ROOT_DIR "" CACHE PATH "Boost root (contains the Boost directory)" )
		if ( NOT BOOST_ROOT_DIR )
			message( SEND_ERROR "No Boost directory specified (BOOST_ROOT_DIR)" )
		endif()
		include_directories(${BOOST_ROOT_DIR})		
	else()
		target_link_libraries(${PROJECT_NAME} Boost::graph)
	endif()
	
	
	if(NOT Eigen3_FOUND)
		set( EIGEN_ROOT_DIR "" CACHE PATH "Eigen root (contains the Eigen directory)" )
		if ( NOT EIGEN_ROOT_DIR )
			message( SEND_ERROR "No Eigen root directory specified (EIGEN_ROOT_DIR)" )
		endif()
		include_directories(${EIGEN_ROOT_DIR})		
	else()
		target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
	endif()






endif()
