option( PLUGIN_STANDARD_QANIMATION "Check to install qAnimation plugin" OFF )

# CloudCompare 'ANIMATION' plugin
if (PLUGIN_STANDARD_QANIMATION)
	option( WITH_FFMPEG_SUPPORT "Check to setup FFMPEG support (to create movies instead of images with qAnimation)" OFF )

	project( QANIMATION_PLUGIN )

    AddPlugin( NAME ${PROJECT_NAME} )

	if (WITH_FFMPEG_SUPPORT)
		add_subdirectory( src/QTFFmpegWrapper )
		include_directories( ${QTFFMPEG_LIB_SOURCE_DIR} )
	endif()

	include_directories( src )

	add_subdirectory( src )

    include( CMakePluginTemplate )

	if (WITH_FFMPEG_SUPPORT)
		target_link_libraries( ${PROJECT_NAME} QTFFMPEG_LIB )
		target_compile_definitions( ${PROJECT_NAME} PRIVATE QFFMPEG_SUPPORT )
	endif()
endif()
