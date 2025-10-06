option( PLUGIN_IO_QPHOTOSCAN "Install qPhotoscan I/O plugin" OFF )

if( PLUGIN_IO_QPHOTOSCAN )
	project( QPHOTOSCAN_IO_PLUGIN )

	AddPlugin( NAME ${PROJECT_NAME} TYPE io )

	add_subdirectory( include )
	add_subdirectory( src )

	set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build quazip shared libraries" FORCE )
	set(QUAZIP_INSTALL OFF CACHE BOOL "Install quazip" FORCE)

	add_subdirectory( extern/quazip EXCLUDE_FROM_ALL )

	target_link_libraries( ${PROJECT_NAME} QuaZip::QuaZip )

endif()
