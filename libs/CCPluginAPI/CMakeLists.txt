project( CCPluginAPI )

add_library( ${PROJECT_NAME} SHARED )

set_target_properties( ${PROJECT_NAME} PROPERTIES
    AUTOMOC OFF
    AUTORCC OFF
    AUTOUIC OFF
    API_STUB_FILE ${CMAKE_CURRENT_LIST_DIR}/stub/ccDefaultPluginInterface.cpp
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

# Document custom property
define_property( TARGET
    PROPERTY
        API_STUB_FILE
    BRIEF_DOCS
        "Path to plugin stub file."
    FULL_DOCS
        "Path to plugin stub file. This file is compiled with each plugin so they can be recognized properly."
)
