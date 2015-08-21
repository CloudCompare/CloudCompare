cmake_minimum_required(VERSION 2.8)

include(ffmpegSupport.cmake)

project( QTFFMPEG_LIB )

file( GLOB header_list *.h )
file( GLOB source_list *.cpp )

if (WIN32)
	include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/win )
endif()

add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} )

#ffmpeg support
target_link_ffmpeg ( ${PROJECT_NAME} )

set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS _CRT_SECURE_NO_WARNINGS )

export_ffmpeg_dlls( ${CLOUDCOMPARE_DEST_FOLDER} )