# ------------------------------------------------------------------------------
# Dxf Lib + CMake support for CloudCompare
# ------------------------------------------------------------------------------

OPTION( OPTION_USE_DXF_LIB "Build with Ribbonsoft's DXF Lib (AutoCAD DXF files support)" OFF )
if( ${OPTION_USE_DXF_LIB} )

	set( DXF_LIB_SRC_DIR "" CACHE PATH "DXF Lib SOURCE directory" )
	if ( NOT DXF_LIB_SRC_DIR )
		message( SEND_ERROR "No DXF lib source dir specified (DXF_LIB_SRC_DIR)" )
	else()
		add_subdirectory(contrib/dxflib)
		include_directories( ${DXF_LIB_SRC_DIR} )
	endif()

endif()

# Link project with dxflib library
function( target_link_DXFLIB ) # 2 arguments: ARGV0 = project name

if( ${OPTION_USE_DXF_LIB} )

	target_link_libraries( ${PROJECT_NAME} DXF_LIB )	
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS CC_DXF_SUPPORT )

endif()

endfunction()
