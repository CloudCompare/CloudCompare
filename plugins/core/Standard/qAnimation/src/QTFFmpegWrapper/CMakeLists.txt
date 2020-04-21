include(ffmpegSupport.cmake)

project( QTFFMPEG_LIB )

file( GLOB header_list *.h )
file( GLOB source_list *.cpp )

add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} )

target_link_libraries( ${PROJECT_NAME}
    Qt5::Gui
)

#ffmpeg support
target_link_ffmpeg ( ${PROJECT_NAME} )

export_ffmpeg_dlls( ${CLOUDCOMPARE_DEST_FOLDER} )
