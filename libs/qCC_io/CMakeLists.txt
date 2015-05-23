cmake_minimum_required(VERSION 2.8)

include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${EXTERNAL_LIBS_INCLUDE_DIR} )
include_directories( ${GLEW_LIB_SOURCE_DIR}/include )
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

# PDMS support
if( ${OPTION_SUPPORT_MAC_PDMS_FORMAT} )
	file( GLOB PDMS_header_list PDMS/*.h )
	file( GLOB PDMS_source_list PDMS/*.cpp )
	list( APPEND header_list ${PDMS_header_list} )
	list( APPEND source_list ${PDMS_source_list} )
endif()

if ( USE_QT5 )
	qt5_wrap_ui( generated_ui_list ${ui_list} )
else()
	# find Qt mocable files
	find_mocable_files( mocable_list ${header_list} )
	qt4_wrap_cpp( moc_list ${mocable_list} )
	qt4_wrap_ui( generated_ui_list ${ui_list} )
endif()	

add_library( ${PROJECT_NAME} SHARED ${header_list} ${source_list} ${moc_list} ${generated_ui_list} )

target_link_libraries( ${PROJECT_NAME} CC_CORE_LIB )
target_link_libraries( ${PROJECT_NAME} CC_FBO_LIB )
target_link_libraries( ${PROJECT_NAME} QCC_DB_LIB )
target_link_libraries( ${PROJECT_NAME} ${EXTERNAL_LIBS_LIBRARIES} )

if ( USE_QT5 )
	qt5_use_modules(${PROJECT_NAME} Core)	
endif()

# contrib. libraries support
target_link_contrib( ${PROJECT_NAME} )

# Default preprocessors
set_default_cc_preproc( ${PROJECT_NAME} )

# Add custom prepocessor definitions
if (WIN32)
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS CC_USE_AS_DLL QCC_IO_USE_AS_DLL )
endif()

# install (shared) lib to specified destinations
if( UNIX )
	install_shared( ${PROJECT_NAME} lib 0 ) #default destination: /usr/lib
else()
foreach( dest ${INSTALL_DESTINATIONS} )
	install_shared( ${PROJECT_NAME} ${dest} 1 )
endforeach()
endif()
