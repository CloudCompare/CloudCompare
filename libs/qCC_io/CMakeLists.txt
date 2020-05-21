project( QCC_IO_LIB )

add_library( ${PROJECT_NAME} SHARED )

add_subdirectory( include )
add_subdirectory( src )
add_subdirectory( ui )

option( OPTION_USE_DXF_LIB "Build with Ribbonsoft's DXF Lib (AutoCAD DXF files support)" ON )
if( ${OPTION_USE_DXF_LIB} )
	set( DXF_LIB_DIR "extern/dxflib" CACHE PATH "DXF lib path" )
	add_subdirectory( "${DXF_LIB_DIR}" )
	target_link_libraries( ${PROJECT_NAME} dxflib )	
endif()

option( OPTION_USE_SHAPE_LIB "Build with ShapeLib (SHP files support)" ON )
if( ${OPTION_USE_SHAPE_LIB} )
    set( SHAPELIB_LIB_DIR "extern/shapelib" CACHE PATH "shapelib lib path" )
	add_subdirectory( "${SHAPELIB_LIB_DIR}" )
	target_link_libraries( ${PROJECT_NAME} shapelib )	
endif()

target_link_libraries( ${PROJECT_NAME}
	QCC_DB_LIB
)

# contrib. libraries support
target_link_contrib( ${PROJECT_NAME} )

# Add custom preprocessor definitions
target_compile_definitions( ${PROJECT_NAME} PRIVATE QCC_IO_LIBRARY_BUILD )

set_target_properties( ${PROJECT_NAME} PROPERTIES
    AUTOUIC ON # FIXME Remove after everything has moved to targets and we can set it globally
	CXX_VISIBILITY_PRESET hidden
)

if ( BUILD_TESTING )
	add_subdirectory( test ) 
endif()

InstallSharedLibrary( TARGET ${PROJECT_NAME} )

