cmake_minimum_required(VERSION 2.8)

include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${EXTERNAL_LIBS_INCLUDE_DIR} )
include_directories( ${GLEW_SOURCE_DIR}/include )
include_directories( ${CC_DLL_SOURCE_DIR}/include )
include_directories( ${triangle_SOURCE_DIR} )
if( MSVC )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/msvc )
endif()

project( QCC_DB_DLL )

file( GLOB header_list *.h)
file( GLOB source_list *.cpp)

# find Qt mocable files
find_mocable_files( mocable_list ${header_list} )
qt4_wrap_cpp( moc_list ${mocable_list} )

add_library( ${PROJECT_NAME} SHARED ${header_list} ${source_list} ${moc_list} )

target_link_libraries( ${PROJECT_NAME} CC_DLL )
target_link_libraries( ${PROJECT_NAME} GLEW )
target_link_libraries( ${PROJECT_NAME} CC_FBO )
target_link_libraries( ${PROJECT_NAME} ${EXTERNAL_LIBS_LIBRARIES} )

# Default preprocessors
set_default_cc_preproc( ${PROJECT_NAME} )

# Add custom prepocessor definitions
set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS USE_GLEW GLEW_STATIC )
if (WIN32)
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS CC_USE_AS_DLL QCC_DB_USE_AS_DLL )
endif()

# install (shared) lib to specified destinations
if( UNIX )
	install_shared( QCC_DB_DLL lib lib ) #default destination: /usr/lib
else()
foreach( dest ${INSTALL_DESTINATIONS} )
	install_shared( QCC_DB_DLL ${dest} ${dest}_debug )
endforeach()
endif()
