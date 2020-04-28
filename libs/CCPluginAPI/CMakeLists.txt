project( CCPluginAPI )

add_library( CCPluginAPI STATIC )

set_target_properties( CCPluginAPI PROPERTIES
    AUTOMOC OFF
    AUTORCC OFF
    AUTOUIC OFF
)

add_subdirectory( include )
add_subdirectory( src )

target_link_libraries( CCPluginAPI
    QCC_IO_LIB
)
