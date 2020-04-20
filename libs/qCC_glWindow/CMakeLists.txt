include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )

project( QCC_GL_LIB )

file( GLOB header_list *.h)
file( GLOB source_list *.cpp)

add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} )

target_link_libraries( ${PROJECT_NAME}
	CCCoreLib
	QCC_DB_LIB
	CC_FBO_LIB )

# OCULUS support
target_link_OCULUS_SDK( ${PROJECT_NAME} )

# Qt
target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL Qt5::OpenGLExtensions)

# Add custom preprocessor definitions
if (OPTION_GL_QUAD_BUFFER_SUPPORT)
	target_compile_definitions( ${PROJECT_NAME} PRIVATE CC_GL_WINDOW_USE_QWINDOW )
endif()
