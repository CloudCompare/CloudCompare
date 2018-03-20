cmake_minimum_required(VERSION 3.0)

option( INSTALL_QADDITIONAL_IO_PLUGIN "Check to install qAdditionalIO plugin" OFF )

if (INSTALL_QADDITIONAL_IO_PLUGIN)
    project( INSTALL_QADDITIONAL_IO_PLUGIN )

	add_subdirectory( filters )
	
    include( ../CMakePluginTpl.cmake )    
endif()
