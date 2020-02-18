cmake_minimum_required( VERSION 3.0 )

option( PLUGIN_IO_QPDAL "Install qPDALIO plugin to read & write LAS files" OFF )

if ( PLUGIN_IO_QPDAL )
	project( QPDAL_IO_PLUGIN )

	include ( ${CC_CMAKE_SCRIPTS_DIR}/CMakePolicies.cmake NO_POLICY_SCOPE )

	set( CC_IS_IO_PLUGIN 1 )
	
	# PDAL library
	find_package( PDAL 1.0.0 REQUIRED CONFIG )
	
	if ( NOT PDAL_FOUND )
		message( SEND_ERROR "PDAL package not found!")
	else()
		include_directories( ${PDAL_INCLUDE_DIRS} )
		link_directories( ${PDAL_LIBRARY_DIRS} )
	endif()

	# Plugin source
	include_directories( src )

	add_subdirectory( src )
	add_subdirectory( ui )

	include( ../../../CMakePluginTpl.cmake )
	
	# Link PDAL
	add_definitions( ${PDAL_DEFINITIONS} )

	target_link_libraries( ${PROJECT_NAME} ${PDAL_LIBRARIES} )

	if( WIN32 )
		if ( MSVC_VERSION GREATER_EQUAL 1900 ) # Visual Studio 2017
			add_definitions( -DWIN32_LEAN_AND_MEAN) 
		endif()

		set( PDAL_DLL_DIR ${PDAL_LIBRARY_DIRS}/../bin )
		
		file( GLOB PDAL_DLL_FILES ${PDAL_DLL_DIR}/pdal*.dll )
		
		copy_files( "${PDAL_DLL_FILES}" ${CLOUDCOMPARE_DEST_FOLDER} )
		
		if( ${OPTION_BUILD_CCVIEWER} )
			copy_files( "${PDAL_DLL_FILES}" ${CCVIEWER_DEST_FOLDER} )
		endif()

		target_link_libraries( ${PROJECT_NAME} "pdal_util" )
	endif()
endif()
