# ------------------------------------------------------------------------------
# FFmpeg+CMake support for CloudCompare
# ------------------------------------------------------------------------------

# Find FFmpeg
set( FFMPEG_INCLUDE_DIR "" CACHE PATH "FFmpeg include directory" )
set( FFMPEG_LIBRARY_DIR "" CACHE PATH "FFmpeg library directory" )

if (WIN32)
	set( FFMPEG_BINARY_DIR "" CACHE PATH "FFmpeg binary directory (where the DLLs are ;-)" )
elseif ( APPLE )
	set( FFMPEG_X264_LIBRARY_DIR "" CACHE PATH "The directory containing the x264 library." )

	if( NOT EXISTS "${FFMPEG_X264_LIBRARY_DIR}" )
		message( SEND_ERROR "x264 library dir does not exist (FFMPEG_X264_LIBRARY_DIR)" )
	endif()
endif()

if( NOT EXISTS "${FFMPEG_INCLUDE_DIR}" )
	message( FATAL_ERROR "FFmpeg include dir does not exist (FFMPEG_INCLUDE_DIR)" )
endif()

if( NOT EXISTS "${FFMPEG_LIBRARY_DIR}" )
    message( FATAL_ERROR "FFmpeg library dir does not exist (FFMPEG_LIBRARY_DIR)" )
endif()

# link project with ffmpeg libraries
function( target_link_ffmpeg ) # 1 argument: ARGV0 = project name
	target_include_directories( ${ARGV0} PRIVATE ${FFMPEG_INCLUDE_DIR} )

	set( FFMPEG_LIBRARIES "" )
	set( FFMPEG_LIBRARIES_ROOT_NAME avutil avcodec avformat swscale ) #unused: avdevice avfilter postproc swresample
	
    foreach( libfile ${FFMPEG_LIBRARIES_ROOT_NAME} )
		if(WIN32)
			list( APPEND FFMPEG_LIBRARIES ${FFMPEG_LIBRARY_DIR}/${libfile}.lib )
		elseif( APPLE )
			list( APPEND FFMPEG_LIBRARIES ${FFMPEG_LIBRARY_DIR}/lib${libfile}.a )
		else()
			list( APPEND FFMPEG_LIBRARIES ${FFMPEG_LIBRARY_DIR}/lib${libfile}.so )
		endif()
	endforeach()

	target_link_libraries( ${ARGV0} ${FFMPEG_LIBRARIES} )

	if( APPLE )
		target_link_libraries( ${ARGV0}
			"-liconv"
			"-L${FFMPEG_X264_LIBRARY_DIR} -lx264"
			"-lz"
			"-framework CoreVideo"
			)
	endif()

	# Required for some C99 defines
	target_compile_definitions( ${ARGV0} PRIVATE __STDC_CONSTANT_MACROS )
    
    unset( FFMPEG_LIBRARIES )
    unset( FFMPEG_LIBRARIES_ROOT_NAME )
endfunction()

function( export_ffmpeg_dlls ) # 1 argument: ARGV0 = destination directory
	if( WIN32 )
		if( EXISTS "${FFMPEG_BINARY_DIR}" )
			set( FFMEG_DLL "")
            
			file( GLOB CODEC_DLL ${FFMPEG_BINARY_DIR}/avcodec*.dll )
			list( APPEND FFMEG_DLL ${CODEC_DLL} )

			file( GLOB FORMAT_DLL ${FFMPEG_BINARY_DIR}/avformat*.dll )
			list( APPEND FFMEG_DLL ${FORMAT_DLL} )

			file( GLOB UTIL_DLL ${FFMPEG_BINARY_DIR}/avutil*.dll )
			list( APPEND FFMEG_DLL ${UTIL_DLL} )

			file( GLOB SW_DLLS ${FFMPEG_BINARY_DIR}/sw*.dll )
			list( APPEND FFMEG_DLL ${SW_DLLS} )

			copy_files( "${FFMEG_DLL}" "${ARGV0}" ) #mind the quotes
		else()
			message( FATAL_ERROR "FFmpeg binary dir does not exist (FFMPEG_BINARY_DIR)" )
		endif()
	endif()
endfunction()
