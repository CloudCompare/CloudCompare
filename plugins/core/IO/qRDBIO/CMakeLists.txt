option( PLUGIN_IO_QRDB "Install RDB 2 I/O plugin" OFF )

if( PLUGIN_IO_QRDB )
	project( QRDB_IO_PLUGIN )
	
	find_package( rdb REQUIRED )

	AddPlugin( NAME ${PROJECT_NAME} TYPE io )

	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )

	target_link_libraries( ${PROJECT_NAME} rdbcpp )
endif()
