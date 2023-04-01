project( QCC_GL_LIB )

add_library( ${PROJECT_NAME} SHARED )

add_subdirectory( include )
add_subdirectory( src )

target_compile_definitions( ${PROJECT_NAME} PRIVATE CCGLWINDOW_LIB_BUILD )

if ( OPTION_GL_QUAD_BUFFER_SUPPORT )
	target_compile_definitions( ${PROJECT_NAME} PRIVATE CC_GL_WINDOW_USE_QWINDOW )
endif()

target_link_libraries( ${PROJECT_NAME}
	QCC_DB_LIB
)

# Oculus support
include( cmake/OculusSupport.cmake )

if ( OPTION_USE_OCULUS_SDK )	
    target_link_OCULUS_SDK( ${PROJECT_NAME} )
endif()

InstallSharedLibrary( TARGET ${PROJECT_NAME} )
