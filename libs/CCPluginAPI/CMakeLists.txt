project( CCPluginAPI )

add_library( ${PROJECT_NAME} SHARED )

set_target_properties( ${PROJECT_NAME} PROPERTIES
    AUTOMOC OFF
    AUTORCC OFF
    AUTOUIC OFF
)

add_subdirectory( include )
add_subdirectory( src )

target_link_libraries( ${PROJECT_NAME}
    QCC_IO_LIB
)

target_compile_definitions( ${PROJECT_NAME} PRIVATE CCPLUGIN_API_LIBRARY_BUILD )

set_target_properties( ${PROJECT_NAME} PROPERTIES
	CXX_VISIBILITY_PRESET hidden
)

InstallSharedLibrary( TARGET ${PROJECT_NAME} )
