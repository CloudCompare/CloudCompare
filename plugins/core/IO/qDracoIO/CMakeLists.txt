option( PLUGIN_IO_QDRACO "Install qDracoIO plugin to read & write DRC files" OFF )

if( PLUGIN_IO_QDRACO )
	project( QDRACO_IO_PLUGIN )

	find_package( Draco REQUIRED )

	AddPlugin( NAME ${PROJECT_NAME} TYPE io )

	if( MSVC )

		target_include_directories( ${PROJECT_NAME} PRIVATE ${DRACO_INCLUDE_DIRS} )
		target_link_libraries( ${PROJECT_NAME} ${Draco_DIR}/../../lib/${DRACO_LIBRARIES}.lib )

	elseif( UNIX )

		target_link_libraries( ${PROJECT_NAME} draco )

	else()

		target_link_libraries( ${PROJECT_NAME} ${DRACO_LIBRARIES} )

	endif()

	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )
 
endif()
