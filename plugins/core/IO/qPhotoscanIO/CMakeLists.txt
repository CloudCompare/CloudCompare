cmake_minimum_required(VERSION 3.0)

option( PLUGIN_IO_QPHOTOSCAN "Check to install qPhotoscan I/O plugin" OFF )

# CloudCompare Photoscan I/O plugin
if (PLUGIN_IO_QPHOTOSCAN)
    
	include( CMakePolicies NO_POLICY_SCOPE )

	include( src/contrib/QuazipLibSupport.cmake )
	
	project( QPHOTOSCAN_IO_PLUGIN )
    
    set( CC_IS_IO_PLUGIN 1 )
    
	include_directories( src )
	
	add_subdirectory( src )

	include( ../../../CMakePluginTpl.cmake )
	
	# Zlib
	target_link_libraries(${PROJECT_NAME} ${ZLIB_LIBRARIES})
	# quazip
	target_link_QUAZIP(${PROJECT_NAME})

endif()
