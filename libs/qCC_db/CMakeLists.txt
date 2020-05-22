project( QCC_DB_LIB )

add_library( ${PROJECT_NAME} SHARED )

add_subdirectory( include )
add_subdirectory( src )

target_link_libraries( ${PROJECT_NAME}
    CCCoreLib
    CC_FBO_LIB
)

target_compile_definitions( ${PROJECT_NAME} PRIVATE QCC_DB_LIBRARY_BUILD )

set_target_properties( ${PROJECT_NAME} PROPERTIES
	CXX_VISIBILITY_PRESET hidden
)

InstallSharedLibrary( TARGET ${PROJECT_NAME} )
