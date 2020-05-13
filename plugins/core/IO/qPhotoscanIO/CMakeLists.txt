option( PLUGIN_IO_QPHOTOSCAN "Install qPhotoscan I/O plugin" OFF )

if( PLUGIN_IO_QPHOTOSCAN )
	project( QPHOTOSCAN_IO_PLUGIN )
		
	AddPlugin( NAME ${PROJECT_NAME} TYPE io )
		
	add_subdirectory( include )
	add_subdirectory( src )

	add_subdirectory( extern/quazip EXCLUDE_FROM_ALL )
	
	target_link_libraries( ${PROJECT_NAME} quazip )
	
	include( CMakePluginTemplate )
endif()
