# ------------------------------------------------------------------------------
# ffmpeg+CMake support for CloudCompare
# ------------------------------------------------------------------------------

# Find ffmpeg
set( FFMPEG_INCLUDE_DIR "" CACHE PATH "ffmpeg include directory" )
set( FFMPEG_LIBRARY_DIR "" CACHE PATH "ffmpeg library directory" )

if (WIN32)
	set( FFMPEG_BINARY_DIR "" CACHE PATH "ffmpeg binary directory (where the DLLs are ;-)" )
elseif ( APPLE )
	set( FFMPEG_X264_LIBRARY_DIR "" CACHE PATH "The directory containing the x264 library." )
	
	if ( NOT FFMPEG_X264_LIBRARY_DIR )
		message( SEND_ERROR "No x264 library directory specified (FFMPEG_X264_LIBRARY_DIR)" )
	endif()
endif()

if ( NOT FFMPEG_INCLUDE_DIR )
	message( SEND_ERROR "No ffmpeg include dir specified (FFMPEG_INCLUDE_DIR)" )
else()
	include_directories( ${FFMPEG_INCLUDE_DIR} )
endif()

# link project with ffmpeg libraries
function( target_link_ffmpeg ) # 1 argument: ARGV0 = project name

	# manual version
	if( FFMPEG_LIBRARY_DIR )
	
		# libraries
		set( FFMPEG_LIBRARIES "" )
		set( FFMPEG_LIBRARIES_ROOT_NAME avutil avcodec avformat swscale ) #unused: avdevice avfilter postproc swresample
		foreach( libfile ${FFMPEG_LIBRARIES_ROOT_NAME} )
			if (WIN32)
				LIST(APPEND FFMPEG_LIBRARIES ${FFMPEG_LIBRARY_DIR}/${libfile}.lib )
			elseif ( APPLE )
				LIST(APPEND FFMPEG_LIBRARIES ${FFMPEG_LIBRARY_DIR}/lib${libfile}.a )
			else()
				LIST(APPEND FFMPEG_LIBRARIES ${FFMPEG_LIBRARY_DIR}/lib${libfile}.so )
			endif()
		endforeach()

		target_link_libraries( ${ARGV0} ${FFMPEG_LIBRARIES} )

		if ( APPLE )
			target_link_libraries( ${ARGV0}
				"-liconv"
				"-L${FFMPEG_X264_LIBRARY_DIR} -lx264"
				"-lz"
				"-framework CoreVideo"
				)
		endif()
		
		# Required for some C99 defines
		set_property( TARGET ${ARGV0} APPEND PROPERTY COMPILE_DEFINITIONS __STDC_CONSTANT_MACROS )

		#set( FFMPEG_LIBRARY_FILES ${FFMPEG_LIBRARIES} CACHE STRING "ffmpeg library files" FORCE )
	else()

		message( SEND_ERROR "No ffmpeg library dir specified (FFMPEG_LIBRARY_DIR)" )

	endif()

endfunction()

function( export_ffmpeg_dlls ) # 1 argument: ARGV0 = destination directory

if (WIN32)

	if (FFMPEG_BINARY_DIR)

		set( FFMPEG_DLL "")
		file (GLOB CODEC_DLL ${FFMPEG_BINARY_DIR}/avcodec*.dll)
		LIST( APPEND FFMPEG_DLL ${CODEC_DLL} )

		file (GLOB FORMAT_DLL ${FFMPEG_BINARY_DIR}/avformat*.dll)
		LIST( APPEND FFMPEG_DLL ${FORMAT_DLL} )

		file (GLOB UTIL_DLL ${FFMPEG_BINARY_DIR}/avutil*.dll)
		LIST( APPEND FFMPEG_DLL ${UTIL_DLL} )
		
		file (GLOB SW_DLLS ${FFMPEG_BINARY_DIR}/sw*.dll)
		LIST( APPEND FFMPEG_DLL ${SW_DLLS} )

		message(STATUS "${FFMPEG_DLL}")
		
		copy_files("${FFMPEG_DLL}" ${ARGV0}) #mind the quotes
	
	else()

		message( SEND_ERROR "No ffmpeg binary dir specified (FFMPEG_BINARY_DIR)" )

	endif()
endif()

endfunction()
