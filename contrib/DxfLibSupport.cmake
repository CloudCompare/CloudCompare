# ------------------------------------------------------------------------------
# Dxf Lib + CMake support for CloudCompare
# ------------------------------------------------------------------------------

OPTION( OPTION_USE_DXF_LIB "Build with Ribbonsoft's DXF Lib (AutoCAD DXF files support)" OFF )
if( ${OPTION_USE_DXF_LIB} )
	set( DXF_LIB_DIR "contrib/dxflib-3.3.4" )
	add_subdirectory( "${DXF_LIB_DIR}" )
	include_directories( "${DXF_LIB_DIR}/src" )
endif()

# Link project with dxflib library
function( target_link_DXFLIB )
	if( ${OPTION_USE_DXF_LIB} )
		target_link_libraries( ${PROJECT_NAME} DXF_LIB )	
		set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS CC_DXF_SUPPORT )
	endif()
endfunction()
