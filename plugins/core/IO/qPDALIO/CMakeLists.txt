option( PLUGIN_IO_QPDAL "Install qPDALIO plugin to read & write LAS files" OFF )

if( PLUGIN_IO_QPDAL )
	project( QPDAL_IO_PLUGIN )
 
	find_package( PDAL 2.0 REQUIRED CONFIG )

	AddPlugin( NAME ${PROJECT_NAME} TYPE io )
	
	add_subdirectory( src )
	add_subdirectory( ui )

	target_link_libraries( ${PROJECT_NAME} ${PDAL_LIBRARIES} )

	if( WIN32 )
		if ( MSVC_VERSION GREATER_EQUAL 1900 ) # Visual Studio 2017
			target_compile_definitions( ${PROJECT_NAME} PRIVATE WIN32_LEAN_AND_MEAN )
		endif()

		set( PDAL_DLL_DIR ${PDAL_LIBRARY_DIRS}/../bin )
		
		file( GLOB PDAL_DLL_FILES ${PDAL_DLL_DIR}/pdal*.dll )
		
		copy_files( "${PDAL_DLL_FILES}" "${CLOUDCOMPARE_DEST_FOLDER}" )
		
		if( ${OPTION_BUILD_CCVIEWER} )
			copy_files( "${PDAL_DLL_FILES}" "${CCVIEWER_DEST_FOLDER}" )
		endif()
		
		unset( PDAL_DLL_DIR )
	endif()
endif()
