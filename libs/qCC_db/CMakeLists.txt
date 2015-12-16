cmake_minimum_required(VERSION 2.8)

include_directories( ${GLEW_LIB_SOURCE_DIR}/include )
include_directories( ${CC_CORE_LIB_SOURCE_DIR}/include )
include_directories( ${CC_FBO_LIB_SOURCE_DIR}/include )
if( MSVC )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/msvc )
endif()

project( QCC_DB_LIB )

file( GLOB header_list *.h)
file( GLOB source_list *.cpp)

add_library( ${PROJECT_NAME} SHARED ${header_list} ${source_list} ${moc_list} )

target_link_libraries( ${PROJECT_NAME} CC_CORE_LIB )
target_link_libraries( ${PROJECT_NAME} GLEW_LIB )
target_link_libraries( ${PROJECT_NAME} CC_FBO_LIB )

if ( USE_QT5 )
	qt5_use_modules(${PROJECT_NAME} Core Gui Widgets OpenGL)
else()
    target_link_libraries( ${PROJECT_NAME} ${QT_LIBRARIES})
endif()

# Add custom prepocessor definitions
set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS USE_GLEW GLEW_STATIC )
if (WIN32)
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS CC_USE_AS_DLL QCC_DB_USE_AS_DLL )
endif()

# install (shared) lib to specified destinations
if( UNIX )
	install_shared( ${PROJECT_NAME} lib 0 ) #default destination: /usr/lib
else()
foreach( dest ${INSTALL_DESTINATIONS} )
	install_shared( ${PROJECT_NAME} ${dest} 1 )
endforeach()
endif()
