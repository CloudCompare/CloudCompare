project( QCC_GL_LIB )

add_library( ${PROJECT_NAME} SHARED )

add_subdirectory( include )
add_subdirectory( src )

target_compile_definitions( ${PROJECT_NAME} PRIVATE CCGLWINDOW_LIB_BUILD )

target_link_libraries( ${PROJECT_NAME}
	QCC_DB_LIB
)

if ( OPTION_USE_OCULUS_SDK )
    target_link_OCULUS_SDK( ${PROJECT_NAME} )
endif()

InstallSharedLibrary( TARGET ${PROJECT_NAME} )
