# ------------------------------------------------------------------------------
# Quick & dirty libfreenect+CMake support for CloudCompare
# ------------------------------------------------------------------------------

set( LIBFREENECT_INCLUDE_DIR "" CACHE PATH "Include directory of libfreenect" )
set( LIBFREENECT_LIBRARY_FILE "" CACHE FILEPATH "Library file of libfreenect" )
if (WIN32)
	set( LIBFREENECT_SHARED_LIBRARY_FILE "" CACHE FILEPATH "Shared library file of libfreenect (dll)" )
endif()

if ( NOT LIBFREENECT_INCLUDE_DIR )
    message( SEND_ERROR "Fill in the libfreenect include folder path (LIBFREENECT_INCLUDE_DIR)" )
endif()
if ( NOT LIBFREENECT_LIBRARY_FILE )
    message( SEND_ERROR "Fill in the libfreenect library file path (LIBFREENECT_LIBRARY_FILE)" )
endif()

include_directories( ${LIBFREENECT_INCLUDE_DIR} )
include_directories( ${LIBFREENECT_INCLUDE_DIR}/libfreenect )

# Link project with libfreenect library and export Dll(s) to specified destinations
function( target_link_libfreenect ) # 2 arguments: ARGV0 = project name / ARGV1 = shared lib export folder

	if( LIBFREENECT_LIBRARY_FILE )

		target_link_libraries( ${PROJECT_NAME} ${LIBFREENECT_LIBRARY_FILE} )

		# post-build events: we need to export freenect.dll as well!
		if ( WIN32 AND ARGV1 )
			if( LIBFREENECT_SHARED_LIBRARY_FILE )
				string( REPLACE \\ / LIBFREENECT_SHARED_LIBRARY_FILE ${LIBFREENECT_SHARED_LIBRARY_FILE} )
				install_ext( FILES ${LIBFREENECT_SHARED_LIBRARY_FILE} ${ARGV1} )
			endif()
		endif()
		
	else()

		message( SEND_ERROR "No libfreenect library file specified (LIBFREENECT_LIBRARY_FILE)" )

	endif()

endfunction()