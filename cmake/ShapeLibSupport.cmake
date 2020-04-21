# ------------------------------------------------------------------------------
# ShapeLib + CMake support for CloudCompare
# ------------------------------------------------------------------------------

option( OPTION_USE_SHAPE_LIB "Build with ShapeLib (SHP files support)" ON )
if( ${OPTION_USE_SHAPE_LIB} )
    set( SHAPELIB_LIB_DIR "extern/shapelib" CACHE PATH "shapelib lib path" )
	add_subdirectory( "${SHAPELIB_LIB_DIR}" )
endif()

# Link project with shapelib library
function( target_link_SHAPE_LIB ) # 2 arguments: ARGV0 = project name
	if( ${OPTION_USE_SHAPE_LIB} )
		target_link_libraries( ${PROJECT_NAME} shapelib )	
	endif()
endfunction()
