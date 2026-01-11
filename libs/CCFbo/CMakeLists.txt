project( CC_FBO_LIB )

qt_add_library( ${PROJECT_NAME} SHARED )

add_subdirectory( include )
add_subdirectory( src )

target_compile_definitions( ${PROJECT_NAME} PRIVATE CCFBO_LIBRARY_BUILD )

target_link_libraries( ${PROJECT_NAME}
	PUBLIC
	Qt6::OpenGL
)

target_include_directories( ${PROJECT_NAME} PUBLIC include)

InstallSharedLibrary( TARGET ${PROJECT_NAME} )
