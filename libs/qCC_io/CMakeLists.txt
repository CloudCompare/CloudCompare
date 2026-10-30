project( QCC_IO_LIB )

add_library( ${PROJECT_NAME} SHARED )

include( cmake/GDALSupport.cmake )

add_subdirectory( include )
add_subdirectory( src )
add_subdirectory( ui )

option( OPTION_USE_DXF_LIB "Build with Ribbonsoft's DXF Lib (AutoCAD DXF files support)" ON )
if( ${OPTION_USE_DXF_LIB} )
	set( DXF_LIB_DIR "extern/dxflib" CACHE PATH "DXF lib path" )
	add_subdirectory( "${DXF_LIB_DIR}" )
	target_link_libraries( ${PROJECT_NAME} PRIVATE dxflib )
endif()

option( OPTION_USE_SHAPE_LIB "Build with ShapeLib (SHP files support)" ON )
option( OPTION_USE_SYSTEM_SHAPE_LIB "Use an installed ShapeLib instead of the bundled copy" OFF )
if( ${OPTION_USE_SHAPE_LIB} )
	if( ${OPTION_USE_SYSTEM_SHAPE_LIB} )
		find_package( shapelib CONFIG QUIET )
		if ( TARGET shapelib::shp )
			target_link_libraries( ${PROJECT_NAME} PUBLIC shapelib::shp )
		else()
			# no (usable) CMake package: the header and library will do
			find_path( SHAPELIB_INCLUDE_DIR shapefil.h REQUIRED )
			find_library( SHAPELIB_LIBRARY NAMES shp shapelib REQUIRED )
			target_include_directories( ${PROJECT_NAME} PUBLIC ${SHAPELIB_INCLUDE_DIR} )
			target_link_libraries( ${PROJECT_NAME} PUBLIC ${SHAPELIB_LIBRARY} )
		endif()
	else()
		set( SHAPELIB_LIB_DIR "extern/shapelib" CACHE PATH "shapelib lib path" )
		add_subdirectory( "${SHAPELIB_LIB_DIR}" )
		target_link_libraries( ${PROJECT_NAME} PUBLIC shapelib )
	endif()
	target_compile_definitions( ${PROJECT_NAME} PUBLIC CC_SHP_SUPPORT )
endif()

target_link_libraries( ${PROJECT_NAME}
	PUBLIC
		QCC_DB_LIB
		Qt::Concurrent
)

if( OPTION_USE_GDAL )
	target_link_GDAL( ${PROJECT_NAME} )
endif()

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
