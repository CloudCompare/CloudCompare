cmake_minimum_required(VERSION 3.0)

option( PLUGIN_IO_QCSV_MATRIX "Check to install qCSVMatrix I/O plugin" OFF )

# CloudCompare CSV Matrix I/O plugin
if (PLUGIN_IO_QCSV_MATRIX)
    project( QCSV_MATRIX_IO_PLUGIN )
    
	include( CMakePolicies NO_POLICY_SCOPE )

    set( CC_IS_IO_PLUGIN 1 )
	
	include_directories( src )
	
	add_subdirectory( src )
	add_subdirectory( ui )
    
    include( ../../../CMakePluginTpl.cmake )
endif()
