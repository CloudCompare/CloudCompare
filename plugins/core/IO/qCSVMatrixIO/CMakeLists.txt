option( PLUGIN_IO_QCSV_MATRIX "Install qCSVMatrix I/O plugin" OFF )

if( PLUGIN_IO_QCSV_MATRIX )
	project( QCSV_MATRIX_IO_PLUGIN )
	
	AddPlugin( NAME ${PROJECT_NAME} TYPE io )
		
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )
endif()
