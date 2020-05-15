option( PLUGIN_STANDARD_QCOMPASS "Check to install Compass plugin" OFF )

if( PLUGIN_STANDARD_QCOMPASS )
	project( QCOMPASS_PLUGIN )

	AddPlugin( NAME ${PROJECT_NAME} )

	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )

	# FIXME This is required becaue we have dependencies outside of the API
	#	(at least ccOverlayDialog*, the picking stuff, and something in "common")
	# We need to determine where those files should live to avoid this.
	target_sources( ${PROJECT_NAME}
		PRIVATE
			# we need ccOverlay classes
			${CloudCompare_SOURCE_DIR}/../common/ccOverlayDialog.h
			${CloudCompare_SOURCE_DIR}/../common/ccOverlayDialog.cpp
			# we also need picking hub classes
			${CloudCompare_SOURCE_DIR}/../common/ccPickingListener.h
			${CloudCompare_SOURCE_DIR}/../common/ccPickingHub.h
			${CloudCompare_SOURCE_DIR}/../common/ccPickingHub.cpp
	)
	target_include_directories( ${PROJECT_NAME}
		PRIVATE
			${CloudCompare_SOURCE_DIR}/../common
	)
endif()
