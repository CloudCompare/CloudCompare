# ------------------------------
# LASLib + CMake support
# ------------------------------

# LASLib include directory
set( LASLIB_INCLUDE_DIR "" CACHE PATH "LASLib include directory" )
set( LASZIP_INCLUDE_DIR "" CACHE PATH "LASZip include directory" )
set( LASLIB_RELEASE_LIBRARY "" CACHE FILEPATH "LASLib release library file" )
set( LASLIB_DEBUG_LIBRARY "" CACHE FILEPATH "LASLib debug library file" )

if ( NOT LASLIB_INCLUDE_DIR OR NOT LASZIP_INCLUDE_DIR )
	message( SEND_ERROR "No LASLib or LASZip include directories specified (LASLIB_INCLUDE_DIR / LASZIP_INCLUDE_DIR)" )
endif()

# link project with LASLib
function( target_link_LASLib ) # 1 argument: ARGV0 = project name
	target_compile_definitions( ${ARGV0} PRIVATE WITH_LASLIB )

	target_include_directories( ${ARGV0}
		PRIVATE
		    ${LASLIB_INCLUDE_DIR}
			${LASZIP_INCLUDE_DIR}
	)

    if( LASLIB_RELEASE_LIBRARY )
        target_link_libraries( ${ARGV0} optimized ${LASLIB_RELEASE_LIBRARY} )
    endif()

    if( CMAKE_CONFIGURATION_TYPES AND LASLIB_DEBUG_LIBRARY )
        target_link_libraries( ${ARGV0} debug ${LASLIB_DEBUG_LIBRARY} )
    endif()
endfunction()
