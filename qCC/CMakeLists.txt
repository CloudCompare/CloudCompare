cmake_minimum_required(VERSION 2.8)

if ( UNIX )
	set( qCC_dest_release bin ) #default destination: /usr/bin
	set( qCC_dest_debug bin/debug )
else()
	set( qCC_dest_release qCC )
	set( qCC_dest_debug qCC_debug )
endif()

# Load plugins
add_subdirectory( plugins )

include_directories( ${GLEW_SOURCE_DIR}/include )
include_directories( ${CC_FBO_SOURCE_DIR}/include )
include_directories( ${CC_DLL_SOURCE_DIR}/include )
include_directories( ${QCC_DB_DLL_SOURCE_DIR} )
if( MSVC )
include_directories( ${QCC_DB_DLL_SOURCE_DIR}/msvc )
endif()
include_directories( ${EXTERNAL_LIBS_INCLUDE_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/db_tree )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/fileIO )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/ui_templates )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/../libs/qxt )
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )
include_directories( ${LIBLAS_INCLUDE_DIR} )
include_directories( ${XIOT_INCLUDE_DIR} )

project( qCC )

set( SAMPLE_PROJECT_VERSION_MAJOR 2 )
set( SAMPLE_PROJECT_VERSION_MINOR 4 )

# TODO: use qxt as a standalone library!
set( QXT_HEADERS ../libs/qxt/qxtspanslider.h ../libs/qxt/qxtspanslider_p.h )
set( QXT_SOURCES ../libs/qxt/qxtspanslider.cpp )

file( GLOB header_list *.h db_tree/*.h fileIO/*.h  ${QXT_HEADERS} )
file( GLOB source_list *.cpp db_tree/*.cpp fileIO/*.cpp fileIO/*.c ${QXT_SOURCES} )

# PDMS support
if( ${OPTION_SUPPORT_MAC_PDMS_FORMAT} )
	file( GLOB PDMS_header_list fileIO/PDMS/*.h )
	file( GLOB PDMS_source_list fileIO/PDMS/*.cpp )
	list( APPEND header_list ${PDMS_header_list} )
	list( APPEND source_list ${PDMS_source_list} )
endif()

file( GLOB ui_list ui_templates/*.ui )
file( GLOB qrc_list *.qrc )
#file( GLOB rc_list *.rc )
file( GLOB txt_list TODO.txt bin_other/history.txt )

# find Qt mocable files
find_mocable_files( mocable_list ${header_list} )
qt4_wrap_cpp( moc_list ${mocable_list} )
qt4_wrap_ui( generated_ui_list ${ui_list} )
qt4_add_resources( generated_qrc_list ${qrc_list} )

# App icon with MSVC
if( MSVC )
	set( rc_list images/icon/qCC.rc )
endif()

if( MSVC )
	#to get rid of the (system) console
	add_executable( ${PROJECT_NAME} WIN32 ${header_list} ${source_list} ${moc_list} ${generated_ui_list} ${generated_qrc_list} ${rc_list} ${resource_list} ${txt_list} )
else()
	add_executable( ${PROJECT_NAME} ${header_list} ${source_list} ${moc_list} ${generated_ui_list} ${generated_qrc_list} ${rc_list} ${resource_list} ${txt_list} )
endif()

target_link_libraries( ${PROJECT_NAME} ${EXTERNAL_LIBS_LIBRARIES} )
target_link_libraries( ${PROJECT_NAME} GLEW )
target_link_libraries( ${PROJECT_NAME} CC_FBO )
target_link_libraries( ${PROJECT_NAME} CC_DLL )
target_link_libraries( ${PROJECT_NAME} QCC_DB_DLL )

# liblas support
target_link_liblas( ${PROJECT_NAME} ${qCC_dest_release} )
# XIOT support
target_link_XIOT( ${PROJECT_NAME} ${qCC_dest_release} ${qCC_dest_debug} )
# E57 support
target_link_LIBE57( ${PROJECT_NAME} )

# Default preprocessors
set_default_cc_preproc( ${PROJECT_NAME} )

# Add custom prepocessor definitions
set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS USE_GLEW GLEW_STATIC )
if( WIN32 )
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS CC_USE_AS_DLL QCC_DB_USE_AS_DLL )
endif()

# ULT files support
OPTION( OPTION_ULT_SUPPORT "Check to support ULT files" OFF )
if( ${OPTION_ULT_SUPPORT} )
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS CC_ULT_SUPPORT )
endif()

# PDMS support
if( ${OPTION_SUPPORT_MAC_PDMS_FORMAT} )
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS CC_PDMS_SUPPORT )
endif()

#set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS_RELEASE XXX) #nothing right now
#set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS_DEBUG XXX) #nothing right now

# App icon with Code::Blocks/MinGW
if( WIN32 )
	if( MINGW )
		add_custom_command( TARGET ${PROJECT_NAME} PRE_BUILD COMMAND windres -i ${CMAKE_CURRENT_SOURCE_DIR}/images/icon/qCC.rc --input-format=rc -o ${CMAKE_CURRENT_BINARY_DIR}/qCC.res -O coff )
	endif()
endif()

# install program
install_ext( TARGETS qCC ${qCC_dest_release} ${qCC_dest_debug} )

# Export 'bilateral' shader files to all install destinations
install_ext( FILES ${CC_FBO_SOURCE_DIR}/bilateral/bilateral.frag ${qCC_dest_release}/shaders ${qCC_dest_debug}/shaders )
install_ext( FILES ${CC_FBO_SOURCE_DIR}/bilateral/bilateral.vert ${qCC_dest_release}/shaders ${qCC_dest_debug}/shaders )

# Auxiliary files
set( auxFiles bin_other/history.txt bin_other/license.txt )

if( WIN32 )
	# Export Qt dlls & plugins
	install_Qt_Dlls( ${qCC_dest_release} ${qCC_dest_debug} )
	install_Qt_ImageFormats( ${qCC_dest_release} ${qCC_dest_debug} )

	# Additional auxiliary file(s)
	list( APPEND auxFiles bin_other/start.bat )
endif()

# Install auxiliary files
foreach( filename ${auxFiles} )
	install_ext( FILES ${filename} ${qCC_dest_release} ${qCC_dest_debug} )
endforeach()
