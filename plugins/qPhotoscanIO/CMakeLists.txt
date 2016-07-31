cmake_minimum_required(VERSION 3.0)

option( INSTALL_QPHOTOSCAN_IO_PLUGIN "Check to install qPhotoscan I/O plugin" OFF )

# CloudCompare Photoscan I/O plugin
if (INSTALL_QPHOTOSCAN_IO_PLUGIN)

	include( contrib/QuazipLibSupport.cmake )
    
	project( QPHOTOSCAN_IO_PLUGIN )
    
    set( CC_IS_IO_PLUGIN 1 )
    
    include( ../CMakePluginTpl.cmake )
	
	# Zlib
	target_link_libraries(${PROJECT_NAME} ${ZLIB_LIBRARIES})
	# quazip
	target_link_QUAZIP(${PROJECT_NAME})
	#target_link_libraries(${PROJECT_NAME} quazip_static)

endif()
