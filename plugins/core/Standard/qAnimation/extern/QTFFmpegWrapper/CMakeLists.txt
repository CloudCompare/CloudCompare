project( QTFFMPEG_LIB )

include( cmake/FFmpegSupport.cmake )

add_library( ${PROJECT_NAME} STATIC )

set_target_properties( ${PROJECT_NAME} PROPERTIES
	AUTOMOC OFF
	AUTORCC OFF
	AUTOUIC OFF
)

target_sources( ${PROJECT_NAME}
	PUBLIC
		${CMAKE_CURRENT_LIST_DIR}/QVideoEncoder.h
		${CMAKE_CURRENT_LIST_DIR}/QVideoEncoder.cpp
)

target_include_directories( ${PROJECT_NAME}
	PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}
		${FFMPEG_INCLUDE_DIR}
)

target_link_libraries( ${PROJECT_NAME}
	Qt5::Gui
)

# FFmpeg support
target_link_ffmpeg( ${PROJECT_NAME} )
export_ffmpeg_dlls( ${CLOUDCOMPARE_DEST_FOLDER} )
