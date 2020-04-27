option( PLUGIN_IO_QCORE "Check to install the CoreIO plugin" ON )

if ( PLUGIN_IO_QCORE )
	project( QCORE_IO_PLUGIN )

    AddPlugin( NAME ${PROJECT_NAME} TYPE io )
	
	include_directories( src )
	include_directories( src/PDMS )

	add_subdirectory( src )
	add_subdirectory( ui )
	
    include( CMakePluginTemplate )
endif()
