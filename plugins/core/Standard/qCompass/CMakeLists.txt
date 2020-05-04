option( PLUGIN_STANDARD_QCOMPASS "Check to install compass plugin" OFF )

if (PLUGIN_STANDARD_QCOMPASS)

	# CloudCompare 'compass' plugin
	project( QCOMPASS_PLUGIN )

    AddPlugin( NAME ${PROJECT_NAME} )

	#we need ccOverlay classes
	file( GLOB CC_PLUGIN_CUSTOM_HEADER_LIST ${CloudCompare_SOURCE_DIR}/../common/ccOverlayDialog*.h )
	file( GLOB CC_PLUGIN_CUSTOM_SOURCE_LIST ${CloudCompare_SOURCE_DIR}/../common/ccOverlayDialog*.cpp )

	#we also need picking hub classes
	list( APPEND CC_PLUGIN_CUSTOM_HEADER_LIST ${CloudCompare_SOURCE_DIR}/../common/ccPickingListener.h )
	list( APPEND CC_PLUGIN_CUSTOM_HEADER_LIST ${CloudCompare_SOURCE_DIR}/../common/ccPickingHub.h )
	list( APPEND CC_PLUGIN_CUSTOM_SOURCE_LIST ${CloudCompare_SOURCE_DIR}/../common/ccPickingHub.cpp )

	add_subdirectory( src )

    include( CMakePluginTemplate )

    # FIXME These are required becaue we have dependencies outside of the api
    #   (at least ccOverlayDialog*, the picking stuff, and something in "common")
    # We need to determine where those files should live to avoid this.
    target_include_directories( ${PROJECT_NAME}
        PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}
            ${CMAKE_CURRENT_LIST_DIR}/src
            ${CloudCompare_SOURCE_DIR}/../common
    )
    
endif()
