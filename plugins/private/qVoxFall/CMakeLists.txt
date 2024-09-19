cmake_minimum_required(VERSION 3.18)

option( PLUGIN_STANDARD_QVOXFALL "Install qVoxFall plugin" ON )

if ( PLUGIN_STANDARD_QVOXFALL )
	project( QVOXFALL_PLUGIN)
	
	AddPlugin( NAME ${PROJECT_NAME} )
	
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )
	
endif()
