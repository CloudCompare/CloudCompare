# ------------------------------------------------------------------------------
# ShapeLib + CMake support for CloudCompare
# ------------------------------------------------------------------------------

OPTION( OPTION_USE_SHAPE_LIB "Build with ShapeLib (SHP files support)" OFF )
if( ${OPTION_USE_SHAPE_LIB} )
	add_subdirectory(contrib/shapelib)
	include_directories( ${SHAPELIB_SOURCE_DIR} )
endif()

# Link project with shapelib library
function( target_link_SHAPE_LIB ) # 2 arguments: ARGV0 = project name
	if( ${OPTION_USE_SHAPE_LIB} )
		target_link_libraries( ${PROJECT_NAME} SHAPELIB )	
		target_compile_definitions( ${PROJECT_NAME} PUBLIC CC_SHP_SUPPORT )
	endif()
endfunction()
