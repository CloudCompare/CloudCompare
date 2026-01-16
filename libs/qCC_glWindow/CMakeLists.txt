project( QCC_GL_LIB )

add_library( ${PROJECT_NAME} SHARED )

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

InstallSharedLibrary( TARGET ${PROJECT_NAME} )
