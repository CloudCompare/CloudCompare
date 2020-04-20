project( QCC_GL_LIB )

add_library( ${PROJECT_NAME} STATIC )

add_subdirectory( include )
add_subdirectory( src )

target_link_libraries( ${PROJECT_NAME}
	QCC_DB_LIB
	Qt5::Core
	Qt5::Gui
	Qt5::OpenGL
	Qt5::OpenGLExtensions
	Qt5::Widgets
)

if ( OPTION_GL_QUAD_BUFFER_SUPPORT )
	target_compile_definitions( ${PROJECT_NAME} PRIVATE CC_GL_WINDOW_USE_QWINDOW )
endif()

if ( OPTION_USE_OCULUS_SDK )
    target_link_OCULUS_SDK( ${PROJECT_NAME} )
endif()
