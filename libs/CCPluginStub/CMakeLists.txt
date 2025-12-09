project( CCPluginStub )

qt_add_library( ${PROJECT_NAME} STATIC )

add_subdirectory( include )
add_subdirectory( src )

target_link_libraries( ${PROJECT_NAME}
	PUBLIC
		QCC_DB_LIB
)

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_VISIBILITY_PRESET hidden )
