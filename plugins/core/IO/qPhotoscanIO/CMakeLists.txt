option( PLUGIN_IO_QPHOTOSCAN "Check to install qPhotoscan I/O plugin" OFF )

# CloudCompare Photoscan I/O plugin
if (PLUGIN_IO_QPHOTOSCAN)
	include( src/contrib/QuazipLibSupport.cmake )
	
	project( QPHOTOSCAN_IO_PLUGIN )
        
    AddPlugin( NAME ${PROJECT_NAME} TYPE io )
    
	include_directories( src )
	
	add_subdirectory( src )

    include( CMakePluginTemplate )
	
	# Zlib
	target_link_libraries(${PROJECT_NAME} ${ZLIB_LIBRARIES})
	# quazip
	target_link_QUAZIP(${PROJECT_NAME})
endif()
