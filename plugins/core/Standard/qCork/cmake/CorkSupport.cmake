# ------------------------------------------------------------------------------
# Quick & dirty Cork+MPIR+CMake support for CloudCompare
# ------------------------------------------------------------------------------

# Cork library (CC fork) https://github.com/cloudcompare/cork
set( CORK_INCLUDE_DIR "" CACHE PATH "Cork include directory" )
set( CORK_RELEASE_LIBRARY_FILE "" CACHE FILEPATH "Cork library file (release mode)" )
if (WIN32)
	set( CORK_DEBUG_LIBRARY_FILE "" CACHE FILEPATH "Cork library file (debug mode)" )
endif()

if ( NOT CORK_INCLUDE_DIR )
	message( SEND_ERROR "No Cork include dir specified (CORK_INCLUDE_DIR)" )
else()
	target_include_directories( ${PROJECT_NAME}
		PRIVATE
		    ${CORK_INCLUDE_DIR}
	)
endif()

set( MPIR_INCLUDE_DIR "" CACHE PATH "MPIR include directory" )
set( MPIR_RELEASE_LIBRARY_FILE "" CACHE FILEPATH "MPIR library file (release mode)" )
if (WIN32)
	set( MPIR_DEBUG_LIBRARY_FILE "" CACHE FILEPATH "MPIR library file (debug mode)" )
endif()

if ( NOT MPIR_INCLUDE_DIR )
	message( SEND_ERROR "No MPIR include dir specified (MPIR_INCLUDE_DIR)" )
else()
	target_include_directories( ${PROJECT_NAME}
		PRIVATE
		    ${MPIR_INCLUDE_DIR}
	)
endif()

# Link project with Cork + MPIR library
function( target_link_cork ) # 1 argument: ARGV0 = project name
	if( CORK_RELEASE_LIBRARY_FILE AND MPIR_RELEASE_LIBRARY_FILE )
		#Release mode only by default
		target_link_libraries( ${ARGV0} optimized ${CORK_RELEASE_LIBRARY_FILE} ${MPIR_RELEASE_LIBRARY_FILE} )

		#optional: debug mode
		if ( CORK_DEBUG_LIBRARY_FILE AND MPIR_DEBUG_LIBRARY_FILE )
			target_link_libraries( ${ARGV0} debug ${CORK_DEBUG_LIBRARY_FILE} ${MPIR_DEBUG_LIBRARY_FILE} )
		endif()
	else()
		message( SEND_ERROR "No Cork or MPIR release library files specified (CORK_RELEASE_LIBRARY_FILE / MPIR_RELEASE_LIBRARY_FILE)" )
	endif()
endfunction()
