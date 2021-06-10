project( CC_FBO_LIB )

add_library( ${PROJECT_NAME} SHARED )

add_subdirectory( include )
add_subdirectory( src )

target_compile_definitions( ${PROJECT_NAME} PRIVATE CCFBO_LIBRARY_BUILD )

target_link_libraries( ${PROJECT_NAME}
	Qt5::OpenGL
	Qt5::OpenGLExtensions
)

InstallSharedLibrary( TARGET ${PROJECT_NAME} )
