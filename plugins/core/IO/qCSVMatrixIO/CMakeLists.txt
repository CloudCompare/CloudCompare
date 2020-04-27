option( PLUGIN_IO_QCSV_MATRIX "Check to install qCSVMatrix I/O plugin" OFF )

# CloudCompare CSV Matrix I/O plugin
if (PLUGIN_IO_QCSV_MATRIX)
    project( QCSV_MATRIX_IO_PLUGIN )
    
    AddPlugin( NAME ${PROJECT_NAME} TYPE io )
	
	include_directories( src )
	
	add_subdirectory( src )
	add_subdirectory( ui )
    
    include( CMakePluginTemplate )
endif()
