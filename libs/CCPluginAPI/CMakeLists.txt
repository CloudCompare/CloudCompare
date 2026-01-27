project( CCPluginAPI )

find_package( Qt6
	COMPONENTS
		Gui
	REQUIRED
)

qt_add_library( ${PROJECT_NAME} SHARED )

add_subdirectory( include )
add_subdirectory( src )
add_subdirectory( ui )

# We only need the includes from the stub
get_target_property( STUB_INCLUDES CCPluginStub INCLUDE_DIRECTORIES )

target_include_directories( ${PROJECT_NAME} PUBLIC "${STUB_INCLUDES}" )

target_link_libraries( ${PROJECT_NAME}
	PUBLIC
		QCC_GL_LIB
		QCC_IO_LIB
)

target_compile_definitions( ${PROJECT_NAME} PRIVATE CCPLUGIN_API_LIBRARY_BUILD )

set_target_properties( ${PROJECT_NAME} PROPERTIES CXX_VISIBILITY_PRESET hidden )

InstallSharedLibrary( TARGET ${PROJECT_NAME} )
