project( CC_FBO_LIB )

add_library( ${PROJECT_NAME} STATIC )

add_subdirectory( include )
add_subdirectory( src )

target_link_libraries( ${PROJECT_NAME} Qt5::Core Qt5::OpenGL Qt5::OpenGLExtensions )
