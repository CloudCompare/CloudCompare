option( PLUGIN_STANDARD_QTREEISO "Install qTreeIso plugin" OFF )

if( PLUGIN_STANDARD_QTREEISO )
	project( QTREEISO_PLUGIN )

	AddPlugin( NAME ${PROJECT_NAME} )
	
	# Find Packages
	set(BOOST_ROOT C:/boost)
	find_package(Boost 1.67.0 REQUIRED)
	
	include_directories(${Boost_INCLUDE_DIRS})
	
	#set( BOOST_ROOT_DIR "" CACHE PATH "Boost root (contains the Boost directory)" )
	#if ( NOT BOOST_ROOT_DIR )
	#	message( SEND_ERROR "No Boost root directory specified (BOOST_ROOT_DIR)" )
	#endif()
	#find_package(Eigen3 REQUIRED)	
	#include_directories(${EIGEN3_INCLUDE_DIR})
	
	
	set( EIGEN_ROOT_DIR "" CACHE PATH "Eigen root (contains the Eigen directory)" )
	if ( NOT EIGEN_ROOT_DIR )
		message( SEND_ERROR "No Eigen root directory specified (EIGEN_ROOT_DIR)" )
	endif()
	
	#find_package (Eigen3 REQUIRED NO_MODULE)

	#include_directories(${EIGEN3_INCLUDE_DIR})
	#include_directories(${BOOST_ROOT_DIR})
	include_directories(${EIGEN_ROOT_DIR})

	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )
endif()
