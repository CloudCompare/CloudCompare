# ------------------------------------------------------------------------------
# Dxf Lib + CMake support for CloudCompare
# ------------------------------------------------------------------------------

OPTION( OPTION_USE_DXF_LIB "Build with Ribbonsoft's DXF Lib (AutoCAD DXF files support)" ON )
if( ${OPTION_USE_DXF_LIB} )
	set( DXF_LIB_DIR "extern/dxflib" CACHE PATH "DXF lib path" )
	add_subdirectory( "${DXF_LIB_DIR}" )
endif()

# Link project with dxflib library
function( target_link_DXFLIB )
	if( ${OPTION_USE_DXF_LIB} )
		target_link_libraries( ${PROJECT_NAME} dxflib )	
	endif()
endfunction()
