cmake_minimum_required(VERSION 3.0)

include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )

include_directories( ${CC_CORE_LIB_SOURCE_DIR}/include )
include_directories( ${CC_FBO_LIB_SOURCE_DIR}/include )
include_directories( ${QCC_DB_LIB_SOURCE_DIR} )
if( MSVC )
   include_directories( ${QCC_DB_LIB_SOURCE_DIR}/msvc )
endif()

project( QCC_GL_LIB )

file( GLOB header_list *.h)
file( GLOB source_list *.cpp)

add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} ${moc_list} )

target_link_libraries( ${PROJECT_NAME} CC_CORE_LIB )
target_link_libraries( ${PROJECT_NAME} QCC_DB_LIB )
target_link_libraries( ${PROJECT_NAME} CC_FBO_LIB )

# OCULUS support
target_link_OCULUS_SDK( ${PROJECT_NAME} )

# Qt
qt5_use_modules(${PROJECT_NAME} Core Gui Widgets OpenGL OpenGLExtensions)

# Add custom preprocessor definitions
# set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS )
if (OPTION_GL_QUAD_BUFFER_SUPPORT)
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS CC_GL_WINDOW_USE_QWINDOW )
endif()
if (WIN32)
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS CC_USE_AS_DLL QCC_DB_USE_AS_DLL )
endif()
