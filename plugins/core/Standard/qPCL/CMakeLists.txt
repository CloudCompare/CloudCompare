option( PLUGIN_STANDARD_QPCL "Install qPCL plugin" OFF )

if( PLUGIN_STANDARD_QPCL )	  
	project( QPCL_PLUGIN )
	
	AddPlugin( NAME ${PROJECT_NAME} )

	add_subdirectory( doc )
	add_subdirectory( include )
	add_subdirectory( src )
	
	add_subdirectory( PclUtils ) # PCL bridge library
	add_subdirectory( PclIO ) # PCL I/O plugin (to load PCD files)
	 
	target_link_libraries( ${PROJECT_NAME} QPCL_PLUGIN_UTILS_LIB )
		
	# Import PCL dlls (if any, WIN32 only)
	include( cmake/ExportPCLDlls.cmake )
	export_PCL_dlls( "${CLOUDCOMPARE_DEST_FOLDER}" )

	include( CMakePluginTemplate )
endif()
