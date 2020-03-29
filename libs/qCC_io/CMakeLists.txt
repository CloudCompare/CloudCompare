cmake_minimum_required(VERSION 3.0)

include( CMakePolicies NO_POLICY_SCOPE )

include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CC_CORE_LIB_SOURCE_DIR}/include )
include_directories( ${CC_FBO_LIB_SOURCE_DIR}/include )
include_directories( ${QCC_DB_LIB_SOURCE_DIR} )
if( MSVC )
   include_directories( ${QCC_DB_LIB_SOURCE_DIR}/msvc )
endif()
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

project( QCC_IO_LIB )

file( GLOB header_list *.h)
file( GLOB source_list *.cpp *.c)
file( GLOB ui_list ui_templates/*.ui )

set( CMAKE_CXX_VISIBILITY_PRESET hidden )

qt5_wrap_ui( generated_ui_list ${ui_list} )
add_library( ${PROJECT_NAME} SHARED ${header_list} ${source_list} ${generated_ui_list} )

target_link_libraries( ${PROJECT_NAME}
	CC_CORE_LIB
	CC_FBO_LIB
	QCC_DB_LIB )

# Qt
target_link_libraries(${PROJECT_NAME} Qt5::Core)	

# contrib. libraries support
target_link_contrib( ${PROJECT_NAME} )

# Add custom preprocessor definitions
target_compile_definitions( ${PROJECT_NAME} PRIVATE QCC_IO_LIBRARY_BUILD )

if( APPLE )
	set_target_properties( ${PROJECT_NAME} PROPERTIES
		INSTALL_RPATH "." )
endif()

# install (shared) lib to specified destinations
if( WIN32 OR APPLE )
	foreach( dest ${INSTALL_DESTINATIONS} )
		install_shared( ${PROJECT_NAME} ${dest} 1 )
	endforeach()
else()
	install_shared( ${PROJECT_NAME} ${CMAKE_INSTALL_LIBDIR}/cloudcompare 0 ) #default destination: /usr/lib
endif()

if(BUILD_TESTING)
	add_subdirectory(Tests)
endif()
