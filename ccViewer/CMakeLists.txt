cmake_minimum_required(VERSION 2.8)

include_directories( ${GLEW_LIB_SOURCE_DIR}/include )
include_directories( ${CC_FBO_LIB_SOURCE_DIR}/include )
include_directories( ${CC_CORE_LIB_SOURCE_DIR}/include )
include_directories( ${QCC_DB_LIB_SOURCE_DIR} )
if( MSVC )
   include_directories( ${QCC_DB_LIB_SOURCE_DIR}/msvc )
endif()
include_directories( ${QCC_IO_LIB_SOURCE_DIR} )
include_directories( ${QCC_GL_LIB_SOURCE_DIR} )
include_directories( ${EXTERNAL_LIBS_INCLUDE_DIR} )
include_directories( ${CloudComparePlugins_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/../qCC )
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

project( ccViewer )

set( VERSION_MAJOR 1 )
set( VERSION_MINOR 32 )
set( VERSION_PATCH 0 )

set( QCC_HEADERS ../qCC/ccOverlayDialog.h ../qCC/ccCameraParamEditDlg.h ../qCC/ccDisplayOptionsDlg.h ../qCC/ccMouse3DContextMenu.h )
set( QCC_SOURCES ../qCC/ccOverlayDialog.cpp ../qCC/ccCameraParamEditDlg.cpp ../qCC/ccDisplayOptionsDlg.cpp ../qCC/ccMouse3DContextMenu.cpp )
set( QCC_DIALOGS ../qCC/ui_templates/cameraParamDlg.ui ../qCC/ui_templates/displayOptionsDlg.ui )

file( GLOB header_list *.h ${QCC_HEADERS} )
file( GLOB source_list *.cpp ${QCC_SOURCES} )
file( GLOB ui_list ui_templates/*.ui ${QCC_DIALOGS} )
file( GLOB qrc_list ../qCC/*.qrc )
file( GLOB rc_list ../qCC/*.rc )

# 3DX support (3dConnexion devices)
if ( ${OPTION_SUPPORT_3DCONNEXION_DEVICES} )
	file( GLOB 3DX_header_list ../qCC/devices/3dConnexion/*.h )
	file( GLOB 3DX_source_list ../qCC/devices/3dConnexion/*.cpp )
	list( APPEND header_list ${3DX_header_list} )
	list( APPEND source_list ${3DX_source_list} )
endif()	

if ( NOT USE_QT5 )

# find Qt mocable files
find_mocable_files( mocable_list ${header_list} )
qt4_wrap_cpp( moc_list ${mocable_list} )
qt4_wrap_ui( generated_ui_list ${ui_list} )
qt4_add_resources( generated_qrc_list ${qrc_list} )

else()

qt5_wrap_ui( generated_ui_list ${ui_list} )
qt5_add_resources( generated_qrc_list ${qrc_list} )

endif()

# App icon with MSVC
if( MSVC )
	set( rc_list images/icon/cc_viewer_icon.rc )
endif()

if( MSVC )
	#to get rid of the (system) console
	add_executable( ${PROJECT_NAME} WIN32 ${header_list} ${source_list} ${moc_list} ${generated_ui_list} ${generated_qrc_list} ${rc_list})
elseif( APPLE )
	add_executable( ${PROJECT_NAME} MACOSX_BUNDLE ${header_list} ${source_list} ${moc_list} ${generated_ui_list} ${generated_qrc_list} ${rc_list})
else()
	add_executable( ${PROJECT_NAME} ${header_list} ${source_list} ${moc_list} ${generated_ui_list} ${generated_qrc_list} ${rc_list})
endif()

target_link_libraries( ${PROJECT_NAME} GLEW_LIB )
target_link_libraries( ${PROJECT_NAME} CC_FBO_LIB )
target_link_libraries( ${PROJECT_NAME} CC_CORE_LIB )
target_link_libraries( ${PROJECT_NAME} QCC_DB_LIB )
target_link_libraries( ${PROJECT_NAME} QCC_IO_LIB )
target_link_libraries( ${PROJECT_NAME} QCC_GL_LIB )
target_link_libraries( ${PROJECT_NAME} ${EXTERNAL_LIBS_LIBRARIES} )

if ( USE_QT5 )
	if (WIN32)
		target_link_libraries( ${PROJECT_NAME} Qt5::WinMain )
	endif()
	qt5_use_modules(${PROJECT_NAME} Core Gui Widgets OpenGL)
endif()

# contrib. libraries support
target_link_contrib( ${PROJECT_NAME} ${CCVIEWER_DEST_FOLDER} )

# 3dConnexion devices support
target_link_3DXWARE( ${PROJECT_NAME} )

# Default preprocessors
set_default_cc_preproc( ${PROJECT_NAME} )

# Add custom preprocessor definitions
set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS CC_VIEWER USE_GLEW GLEW_STATIC )
if (WIN32)
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS CC_USE_AS_DLL QCC_DB_USE_AS_DLL QCC_IO_USE_AS_DLL )
	if (MSVC)
		SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LINK_FLAGS " /MANIFEST:NO")
	endif()
endif()

# App icon with Code::Blocks/MinGW
if( WIN32 )
	if( MINGW )
		add_custom_command( TARGET ${PROJECT_NAME} PRE_BUILD COMMAND windres -i ${CMAKE_CURRENT_SOURCE_DIR}/images/icon/cc_viewer_icon.rc --input-format=rc -o ${CMAKE_CURRENT_BINARY_DIR}/ccViewer.res -O coff )
	endif()
endif()

# install program
install_ext( TARGETS ccViewer ${CCVIEWER_DEST_FOLDER} )

# Auxiliary files
set( auxFiles bin_other/license.txt )

if( WIN32 )
	# Export Qt dlls
	install_Qt_Dlls( ${CCVIEWER_DEST_FOLDER} )
	install_Qt5_plugins( ${CCVIEWER_DEST_FOLDER} )

	# Additional auxiliary file(s)
	list( APPEND auxFiles bin_other/start.bat )
endif()

# Install auxiliary files
foreach( filename ${auxFiles} )
	install_ext( FILES ${filename} ${CCVIEWER_DEST_FOLDER} )
endforeach()

# Put together our App bundle on Mac OS X
if( APPLE )
   set( CCVIEWER_MAC_APP_NAME ${PROJECT_NAME}.app )
   set( CCVIEWER_MAC_BASE_DIR ${CMAKE_INSTALL_PREFIX}/${CCVIEWER_DEST_FOLDER}/${CCVIEWER_MAC_APP_NAME} CACHE INTERNAL "ccViewer bundle dir.")

   set_property( TARGET ${PROJECT_NAME} PROPERTY MACOSX_BUNDLE_INFO_PLIST ${CMAKE_CURRENT_SOURCE_DIR}/Mac/ccViewer.plist )

   set( MACOSX_BUNDLE_ICON_FILE cc_viewer_icon.icns ) 
   set( MACOSX_BUNDLE_SHORT_VERSION_STRING "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}" )
   set( MACOSX_BUNDLE_LONG_VERSION_STRING "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}" )
   set( MACOSX_BUNDLE_BUNDLE_VERSION "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}" )

   add_subdirectory( Mac )
endif( APPLE )

# Export common shader files to all install destinations
if( APPLE )
   install( FILES ${CMAKE_CURRENT_SOURCE_DIR}/../qCC/shaders/ColorRamp/color_ramp.frag DESTINATION ${CCVIEWER_MAC_BASE_DIR}/Contents/Shaders/ColorRamp )
else()
   install_ext( FILES ${CMAKE_CURRENT_SOURCE_DIR}/../qCC/shaders/ColorRamp/color_ramp.frag ${CCVIEWER_DEST_FOLDER} /shaders/ColorRamp )
endif()
