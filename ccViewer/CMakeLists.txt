cmake_minimum_required(VERSION 3.0)

project( ccViewer VERSION 1.37.0 )

include_directories( ${CC_FBO_LIB_SOURCE_DIR}/include )
include_directories( ${CC_CORE_LIB_SOURCE_DIR}/include )
include_directories( ${QCC_DB_LIB_SOURCE_DIR} )
if( MSVC )
   include_directories( ${QCC_DB_LIB_SOURCE_DIR}/msvc )
endif()
include_directories( ${QCC_IO_LIB_SOURCE_DIR} )
include_directories( ${QCC_GL_LIB_SOURCE_DIR} )

include_directories( ${CloudComparePlugins_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/../qCC )
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

file( GLOB header_list *.h )
file( GLOB source_list *.cpp )

file( GLOB ui_list ui_templates/*.ui )
file( GLOB qrc_list ../qCC/*.qrc )
file( GLOB rc_list ../qCC/*.rc )

include( CMakeIncludeQCCFiles.cmake )

# 3DX support (3dConnexion devices)
if ( ${OPTION_SUPPORT_3DCONNEXION_DEVICES} )
	add_subdirectory( ../qCC/devices/3dConnexion devices/3dConnexion )
endif()	

# Gamepad support (requires Qt 5.7+)
if ( ${OPTION_SUPPORT_GAMEPADS} )
	add_subdirectory( ../qCC/devices/gamepad devices/gamepad )
endif()	

qt5_wrap_ui( generated_ui_list ${ui_list} )
qt5_add_resources( generated_qrc_list ${qrc_list} )

if( MSVC )
	# App icon with MSVC
	set( rc_list images/icon/cc_viewer_icon.rc )

	#to get rid of the (system) console
	add_executable( ${PROJECT_NAME} WIN32 ${header_list} ${source_list} ${generated_ui_list} ${generated_qrc_list} ${rc_list})
elseif( APPLE )
	add_executable( ${PROJECT_NAME} MACOSX_BUNDLE ${header_list} ${source_list} ${generated_ui_list} ${generated_qrc_list} ${rc_list})
else()
	add_executable( ${PROJECT_NAME} ${header_list} ${source_list} ${generated_ui_list} ${generated_qrc_list} ${rc_list})
endif()

target_link_libraries( ${PROJECT_NAME} CC_FBO_LIB )
target_link_libraries( ${PROJECT_NAME} CC_CORE_LIB )
target_link_libraries( ${PROJECT_NAME} QCC_DB_LIB )
target_link_libraries( ${PROJECT_NAME} QCC_IO_LIB )
target_link_libraries( ${PROJECT_NAME} QCC_GL_LIB )

# Qt
target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL)
if (WIN32)
	target_link_libraries( ${PROJECT_NAME} Qt5::WinMain )
endif()

# contrib. libraries support
if ( APPLE )
	target_link_contrib( ${PROJECT_NAME} ${CLOUDCOMPARE_MAC_FRAMEWORK_DIR} )
elseif( UNIX )
	target_link_contrib( ${PROJECT_NAME} ${CMAKE_INSTALL_LIBDIR}/cloudcompare )
else()
	target_link_contrib( ${PROJECT_NAME} ${CCVIEWER_DEST_FOLDER} )
endif()

# 3dConnexion devices support
if ( ${OPTION_SUPPORT_3DCONNEXION_DEVICES} )
	target_link_3DXWARE( ${PROJECT_NAME} )
endif()

# Gamepad support (requires Qt 5.7+)
if ( ${OPTION_SUPPORT_GAMEPADS} )
	target_link_GAMEPADS( ${PROJECT_NAME} )
endif()

# Add custom preprocessor definitions
set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS CC_VIEWER )
if (OPTION_GL_QUAD_BUFFER_SUPPORT)
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS CC_GL_WINDOW_USE_QWINDOW )
endif()
if (WIN32)
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS CC_USE_AS_DLL QCC_DB_USE_AS_DLL QCC_IO_USE_AS_DLL )
	if (MSVC)
		SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LINK_FLAGS " /MANIFEST:NO")
	endif()
endif()

# App icon with Code::Blocks/MinGW
if( WIN32 )
	if( MINGW )
		add_custom_command( TARGET ${PROJECT_NAME} PRE_BUILD COMMAND ${CMAKE_RC_COMPILER} -i ${CMAKE_CURRENT_SOURCE_DIR}/images/icon/cc_viewer_icon.rc --input-format=rc -o ${CMAKE_CURRENT_BINARY_DIR}/ccViewer.res -O coff )
	endif()
endif()

# install program
install_ext( TARGETS ccViewer ${CCVIEWER_DEST_FOLDER} )

# Auxiliary files
set( auxFiles bin_other/license.txt )

if( WIN32 )
	# Export Qt dlls
	install_Qt_Dlls( ${CCVIEWER_DEST_FOLDER} ${OPTION_SUPPORT_GAMEPADS} )
	install_Qt5_plugins( ${CCVIEWER_DEST_FOLDER} )

	# Additional auxiliary file(s)
	list( APPEND auxFiles bin_other/start.bat )
endif()

# Install auxiliary files
if(APPLE OR WIN32) # For Linux it's already installed in by qCC
	foreach( filename ${auxFiles} )
		install_ext( FILES ${filename} ${CCVIEWER_DEST_FOLDER} )
	endforeach()
endif()

# Put together our App bundle on Mac OS X
if( APPLE )
	set_target_properties( ${PROJECT_NAME} PROPERTIES
		INSTALL_RPATH "@executable_path/../Frameworks"
		MACOSX_BUNDLE_INFO_PLIST ${CMAKE_CURRENT_SOURCE_DIR}/Mac/ccViewer.plist )

	set( MACOSX_BUNDLE_ICON_FILE cc_viewer_icon.icns )
	set( MACOSX_BUNDLE_SHORT_VERSION_STRING "${PROJECT_VERSION}" )
	set( MACOSX_BUNDLE_LONG_VERSION_STRING "${PROJECT_VERSION}" )
	set( MACOSX_BUNDLE_BUNDLE_VERSION "${PROJECT_VERSION}" )

	add_subdirectory( Mac )
endif( APPLE )

# Export common shader files to all install destinations
if( APPLE )
	install( FILES ${CMAKE_CURRENT_SOURCE_DIR}/../qCC/shaders/ColorRamp/color_ramp.frag DESTINATION ${CCVIEWER_MAC_BASE_DIR}/Contents/Shaders/ColorRamp )
elseif( WIN32 ) # For Linux it's already installed in by qCC
	install_ext( FILES ${CMAKE_CURRENT_SOURCE_DIR}/../qCC/shaders/ColorRamp/color_ramp.frag ${CCVIEWER_DEST_FOLDER} /shaders/ColorRamp )
endif()
