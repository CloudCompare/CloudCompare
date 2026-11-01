
option( PLUGIN_IO_QE57 "Install qE57_IO plugin to read & write E57 files using libE57Format" OFF )

if ( PLUGIN_IO_QE57 )
	project( QE57_IO_PLUGIN )

	AddPlugin( NAME ${PROJECT_NAME} TYPE io )

	add_subdirectory( include )
	add_subdirectory( src )
	option( PLUGIN_IO_QE57_USE_SYSTEM_LIBE57FORMAT "Use an installed libE57Format instead of the bundled copy" OFF )
	if ( PLUGIN_IO_QE57_USE_SYSTEM_LIBE57FORMAT )
		find_package( E57Format CONFIG REQUIRED )
	else()
		add_subdirectory( extern/libE57Format EXCLUDE_FROM_ALL )

		set_target_properties( E57Format PROPERTIES
			AUTOMOC OFF
			AUTORCC OFF
			AUTOUIC OFF
		)
	endif()


	target_link_libraries( ${PROJECT_NAME} E57Format )
	
	if ( USING_STATIC_XERCES )
		target_compile_definitions( ${PROJECT_NAME} PRIVATE XERCES_STATIC_LIBRARY )
	endif() 
endif()
