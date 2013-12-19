cmake_minimum_required(VERSION 2.8)

if ( UNIX )
	set( ccViewer_dest_release bin ) #default UNIX destination: /usr/bin
	set( ccViewer_dest_debug bin/debug )
else()
	set( ccViewer_dest_release ccViewer )
	set( ccViewer_dest_debug ccViewer_debug )
endif()

include_directories( ${GLEW_SOURCE_DIR}/include )
include_directories( ${CC_FBO_SOURCE_DIR}/include )
include_directories( ${CC_DLL_SOURCE_DIR}/include )
include_directories( ${QCC_DB_DLL_SOURCE_DIR} )
if( MSVC )
   include_directories( ${QCC_DB_DLL_SOURCE_DIR}/msvc )
endif()
include_directories( ${EXTERNAL_LIBS_INCLUDE_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/../qCC )
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )
include_directories( ${LIBLAS_INCLUDE_DIR} )
include_directories( ${XIOT_INCLUDE_DIR} )

project( ccViewer )

set( VERSION_MAJOR 1 )
set( VERSION_MINOR 28 )
set( VERSION_PATCH 0 )

set( QCC_HEADERS ../qCC/ccGLUtils.h ../qCC/ccGLWindow.h ../qCC/ccGuiParameters.h ../qCC/ccOverlayDialog.h ../qCC/ccRenderingTools.h ../qCC/ccCameraParamEditDlg.h ../qCC/ccDisplayOptionsDlg.h ../qCC/ccShiftAndScaleCloudDlg.h ../qCC/fileIO/*.h ../qCC/ccMouse3DContextMenu.h ../qCC/ccCoordinatesShiftManager.h)
set( QCC_SOURCES ../qCC/ccGLUtils.cpp ../qCC/ccGLWindow.cpp ../qCC/ccGuiParameters.cpp ../qCC/ccOverlayDialog.cpp ../qCC/ccRenderingTools.cpp ../qCC/ccCameraParamEditDlg.cpp ../qCC/ccDisplayOptionsDlg.cpp ../qCC/ccShiftAndScaleCloudDlg.cpp ../qCC/fileIO/*.cpp ../qCC/fileIO/*.c  ../qCC/ccMouse3DContextMenu.cpp ../qCC/ccCoordinatesShiftManager.cpp)
set( QCC_DIALOGS ../qCC/ui_templates/cameraParamDlg.ui ../qCC/ui_templates/displayOptionsDlg.ui ../qCC/ui_templates/shiftAndScaleCloudDlg.ui ../qCC/ui_templates/open*.ui ../qCC/ui_templates/saveAsciiFileDlg.ui )

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

# PDMS support
if( ${OPTION_SUPPORT_MAC_PDMS_FORMAT} )
	file( GLOB PDMS_header_list ../qCC/fileIO/PDMS/*.h )
	file( GLOB PDMS_source_list ../qCC/fileIO/PDMS/*.cpp )
	list( APPEND header_list ${PDMS_header_list} )
	list( APPEND source_list ${PDMS_source_list} )
endif()

# find Qt mocable files
find_mocable_files( mocable_list ${header_list} )
qt4_wrap_cpp( moc_list ${mocable_list} )
qt4_wrap_ui( generated_ui_list ${ui_list} )
qt4_add_resources( generated_qrc_list ${qrc_list} )

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

target_link_libraries( ${PROJECT_NAME} ${EXTERNAL_LIBS_LIBRARIES} )
target_link_libraries( ${PROJECT_NAME} GLEW )
target_link_libraries( ${PROJECT_NAME} CC_FBO )
target_link_libraries( ${PROJECT_NAME} CC_DLL )
target_link_libraries( ${PROJECT_NAME} QCC_DB_DLL )

# liblas support
target_link_liblas( ${PROJECT_NAME} ${ccViewer_dest_release} )
# XIOT support
target_link_XIOT( ${PROJECT_NAME} ${ccViewer_dest_release} ${ccViewer_dest_debug} )
# E57 support
target_link_LIBE57( ${PROJECT_NAME} )
# 3dConnexion devices support
target_link_3DXWARE( ${PROJECT_NAME} )
#DXF support
target_link_DXFLIB( ${PROJECT_NAME} )
#FBX support
target_link_FBX_SDK( ${PROJECT_NAME} )

# Default preprocessors
set_default_cc_preproc( ${PROJECT_NAME} )

# Add custom prepocessor definitions
set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS CC_VIEWER USE_GLEW GLEW_STATIC )
if (WIN32)
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS CC_USE_AS_DLL QCC_DB_USE_AS_DLL )
endif()

# PDMS support
if( ${OPTION_SUPPORT_MAC_PDMS_FORMAT} )
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS CC_PDMS_SUPPORT )
endif()

# App icon with Code::Blocks/MinGW
if( WIN32 )
	if( MINGW )
		add_custom_command( TARGET ${PROJECT_NAME} PRE_BUILD COMMAND windres -i ${CMAKE_CURRENT_SOURCE_DIR}/images/icon/ccViewer.rc --input-format=rc -o ${CMAKE_CURRENT_BINARY_DIR}/ccViewer.res -O coff )
	endif()
endif()

# install program
install_ext( TARGETS ccViewer ${ccViewer_dest_release} ${ccViewer_dest_debug} )

# Auxiliary files
set( auxFiles bin_other/license.txt )

if( WIN32 )
	# Export Qt dlls
	install_Qt_Dlls( ${ccViewer_dest_release} ${ccViewer_dest_debug} )

	# Additional auxiliary file(s)
	list( APPEND auxFiles bin_other/start.bat )
endif()

# Install auxiliary files
foreach( filename ${auxFiles} )
	install_ext( FILES ${filename} ${ccViewer_dest_release} ${ccViewer_dest_debug} )
endforeach()

# Put together our App bundle on Mac OS X
if( APPLE )
   set( MAC_APP_NAME ${PROJECT_NAME}.app )
   set( MAC_BUNDLE_MACOS_DIR ${CMAKE_INSTALL_PREFIX}/${ccViewer_dest_release}/${MAC_APP_NAME}/Contents/MacOS )
   set( MAC_BUNDLE_RESOURCES_DIR ${CMAKE_INSTALL_PREFIX}/${ccViewer_dest_release}/${MAC_APP_NAME}/Contents/Resources )
   set( MAC_BUNDLE_PLUGIN_DIR ${CMAKE_INSTALL_PREFIX}/${ccViewer_dest_release}/${MAC_APP_NAME}/Contents/Plugins )
   set( MAC_BUNDLE_LIB_DIR ${CMAKE_INSTALL_PREFIX}/${ccViewer_dest_release}/${MAC_APP_NAME}/Contents/Libs )
   set( MAC_BUNDLE_SHADERS_DIR ${CMAKE_INSTALL_PREFIX}/${ccViewer_dest_release}/${MAC_APP_NAME}/Contents/Shaders )

   set_property( TARGET ${PROJECT_NAME} PROPERTY MACOSX_BUNDLE_INFO_PLIST ${CMAKE_CURRENT_SOURCE_DIR}/Mac/ccViewer.plist )
   set_source_files_properties( images/icon/cc_viewer_icon.icns PROPERTIES MACOSX_PACKAGE_LOCATION Resources )

   set( MACOSX_BUNDLE_ICON_FILE cc_viewer_icon.icns ) 
   set( MACOSX_BUNDLE_SHORT_VERSION_STRING "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}" )
   set( MACOSX_BUNDLE_LONG_VERSION_STRING "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}" )
   set( MACOSX_BUNDLE_BUNDLE_VERSION "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}" )

   add_subdirectory( Mac )
endif( APPLE )

# Export common shader files to all install destinations
if( APPLE )
   install( FILES ${CMAKE_CURRENT_SOURCE_DIR}/../qCC/shaders/ColorRamp/color_ramp.frag DESTINATION ${MAC_BUNDLE_SHADERS_DIR}/ColorRamp )
else()
   install_ext( FILES ${CMAKE_CURRENT_SOURCE_DIR}/../qCC/shaders/ColorRamp/color_ramp.frag ${ccViewer_dest_release}/shaders/ColorRamp ${ccViewer_dest_debug}/shaders/ColorRamp )
endif()
