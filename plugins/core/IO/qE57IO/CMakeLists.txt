
option( PLUGIN_IO_QE57 "Install qE57_IO plugin to read & write E57 files using libE57Format" OFF )

if ( PLUGIN_IO_QE57 )
	project( QE57_IO_PLUGIN )

	AddPlugin( NAME ${PROJECT_NAME} TYPE io )

	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( extern/libE57Format EXCLUDE_FROM_ALL )
	
	set_target_properties( E57Format PROPERTIES
		AUTOMOC OFF
		AUTORCC OFF
		AUTOUIC OFF
	)
			
	target_link_libraries( ${PROJECT_NAME} E57Format )
	
	if ( USING_STATIC_XERCES )
		target_compile_definitions( ${PROJECT_NAME} PRIVATE XERCES_STATIC_LIBRARY )
	endif() 
endif()
