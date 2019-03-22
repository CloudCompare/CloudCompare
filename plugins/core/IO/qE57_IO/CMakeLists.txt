cmake_minimum_required( VERSION 3.1 )

option( PLUGIN_IO_QE57 "Install qE57_IO plugin to read & write E57 files using libE57Format" OFF )

if ( PLUGIN_IO_QE57 )
	project( qE57_IO )

	find_package( XercesC REQUIRED )
	
	include_directories( ${XercesC_INCLUDE_DIRS} )
	include_directories( extern/libE57Format/include )
	include_directories( src )

	add_subdirectory( src )
	add_subdirectory( extern/libE57Format )
	
	include( ../../../CMakePluginTpl.cmake )
	
	target_link_libraries( ${PROJECT_NAME} E57Format ${XercesC_LIBRARIES} )
endif()
