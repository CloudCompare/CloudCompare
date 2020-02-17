cmake_minimum_required(VERSION 3.0)

option( PLUGIN_STANDARD_QANIMATION "Check to install qAnimation plugin" OFF )

# CloudCompare 'ANIMATION' plugin
if (PLUGIN_STANDARD_QANIMATION)
	option( WITH_FFMPEG_SUPPORT "Check to setup FFMPEG support (to create movies instead of images with qAnimation)" OFF )

	project( QANIMATION_PLUGIN )

	include ( ${CC_CMAKE_SCRIPTS_DIR}/CMakePolicies.cmake NO_POLICY_SCOPE )

	if (WITH_FFMPEG_SUPPORT)
		add_subdirectory( src/QTFFmpegWrapper )
		include_directories( ${QTFFMPEG_LIB_SOURCE_DIR} )
	endif()

	include_directories( src )

	add_subdirectory( src )
	add_subdirectory( ui )

	include( ../../../CMakePluginTpl.cmake )

	if (WITH_FFMPEG_SUPPORT)
		target_link_libraries( ${PROJECT_NAME} QTFFMPEG_LIB )
		set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS QFFMPEG_SUPPORT )
	endif()
endif()
