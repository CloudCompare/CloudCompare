option( PLUGIN_IO_QPHOTOSCAN "Install qPhotoscan I/O plugin" OFF )

if( PLUGIN_IO_QPHOTOSCAN )
	project( QPHOTOSCAN_IO_PLUGIN )
		
	AddPlugin( NAME ${PROJECT_NAME} TYPE io )
		
	add_subdirectory( include )
	add_subdirectory( src )

	add_subdirectory( extern/quazip EXCLUDE_FROM_ALL )
	
	target_link_libraries( ${PROJECT_NAME} QuaZip::QuaZip )
	
	# copy Qt 6 Core5Compatibility DLL
    find_package(Qt6 REQUIRED COMPONENTS Core5Compat)
	get_target_property( Qt6_CORE5COMPAT_LIB_LOCATION Qt6::Core5Compat LOCATION_RELEASE )
	copy_files(${Qt6_CORE5COMPAT_LIB_LOCATION} "${CLOUDCOMPARE_DEST_FOLDER}" 0) #mind the quotes!
	if (${OPTION_BUILD_CCVIEWER})
		copy_files(${Qt6_CORE5COMPAT_LIB_LOCATION} "${CCVIEWER_DEST_FOLDER}" 0) #mind the quotes!
	endif()
	
	if( CMAKE_CONFIGURATION_TYPES )
		get_target_property( Qt6_CORE5COMPAT_DEBUG_LIB_LOCATION Qt6::Core5Compat LOCATION_DEBUG )
		copy_files(${Qt6_CORE5COMPAT_DEBUG_LIB_LOCATION} "${CLOUDCOMPARE_DEST_FOLDER}" 2) #mind the quotes!
		if (${OPTION_BUILD_CCVIEWER})
			copy_files(${Qt6_CORE5COMPAT_DEBUG_LIB_LOCATION} "${CCVIEWER_DEST_FOLDER}" 2) #mind the quotes!
		endif()
	endif()


endif()
