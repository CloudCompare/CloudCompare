cmake_minimum_required(VERSION 2.8)

# Eliminate a warning when building in Windows that relates
# to static linking of Qt executables to qtmain.lib.
# This policy was introduced in CMake version 2.8.11.
# CMake version 2.8.11.2 warns when the policy is not set
# and uses OLD behavior.
if(POLICY CMP0020)
  cmake_policy(SET CMP0020 OLD)
endif()

option( INSTALL_QANIMATION_PLUGIN "Check to install qAnimation plugin" OFF )

if (INSTALL_QANIMATION_PLUGIN)

	option( WITH_FFMPEG_SUPPORT "Check to setup FFMPEG support (to create movies instead of images with qAnimation)" OFF )
	if (WITH_FFMPEG_SUPPORT)
		add_subdirectory (QTFFmpegWrapper)
	endif()
	
	include_directories( ${QTFFMPEG_LIB_SOURCE_DIR} )

	#CloudCompare 'ANIMATION' plugin
	project( QANIMATION_PLUGIN )

	include( ../CMakePluginTpl.cmake )
	
	if (WITH_FFMPEG_SUPPORT)
		target_link_libraries( ${PROJECT_NAME} QTFFMPEG_LIB )
		set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS QFFMPEG_SUPPORT )
	endif()

endif()
