cmake_minimum_required(VERSION 3.0)

include_directories( ${CC_CORE_LIB_SOURCE_DIR}/include )
include_directories( ${CC_FBO_LIB_SOURCE_DIR}/include )

project( QCC_DB_LIB )

file( GLOB header_list *.h)
file( GLOB source_list *.cpp)

set( CMAKE_CXX_VISIBILITY_PRESET hidden )

add_library( ${PROJECT_NAME} SHARED ${header_list} ${source_list} )

target_link_libraries( ${PROJECT_NAME}
	CC_CORE_LIB
	CC_FBO_LIB )

# Qt
target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL)

# Add custom preprocessor definitions
target_compile_definitions( ${PROJECT_NAME} PRIVATE QCC_DB_LIBRARY_BUILD )

# install (shared) lib to specified destinations
if( WIN32 OR APPLE )
	foreach( dest ${INSTALL_DESTINATIONS} )
		install_shared( ${PROJECT_NAME} ${dest} 1 )
	endforeach()
else()
	install_shared( ${PROJECT_NAME} ${CMAKE_INSTALL_LIBDIR}/cloudcompare 0 ) #default destination: /usr/lib
endif()
