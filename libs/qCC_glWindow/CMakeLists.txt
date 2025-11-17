project( QCC_GL_LIB )

qt_add_library( ${PROJECT_NAME} SHARED )

add_subdirectory( include )
add_subdirectory( src )

target_compile_definitions( ${PROJECT_NAME} PRIVATE CCGLWINDOW_LIB_BUILD )

target_link_libraries( ${PROJECT_NAME}
    PUBLIC
	    QCC_DB_LIB
        Qt6::OpenGLWidgets
    PRIVATE
        CC_FBO_LIB
)

# Oculus support
include( cmake/OculusSupport.cmake )

if ( OPTION_USE_OCULUS_SDK )
    target_link_OCULUS_SDK( ${PROJECT_NAME} )
endif()

InstallSharedLibrary( TARGET ${PROJECT_NAME} )
