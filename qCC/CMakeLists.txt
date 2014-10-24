cmake_minimum_required(VERSION 2.8)

# Eliminate a warning when building in Windows that relates
# to static linking of Qt executables to qtmain.lib.
# This policy was introduced in CMake version 2.8.11.
# CMake version 2.8.11.2 warns when the policy is not set
# and uses OLD behavior.
if(POLICY CMP0020)
  cmake_policy(SET CMP0020 OLD)
endif()

project( CloudCompare )

set( VERSION_MAJOR 2 )
set( VERSION_MINOR 6 )
set( VERSION_PATCH 0 )

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
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/db_tree )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/ui_templates )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/../libs/qxt )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/../libs/qcustomplot )
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

# QCustomPlot
set( QCUSTOMPLOT_HEADERS ../libs/qcustomplot/qcustomplot.h )
set( QCUSTOMPLOT_SOURCES ../libs/qcustomplot/qcustomplot.cpp )

file( GLOB header_list *.h db_tree/*.h ${QXT_HEADERS} ${QCUSTOMPLOT_HEADERS} )
file( GLOB source_list *.cpp db_tree/*.cpp ${QXT_SOURCES} ${QCUSTOMPLOT_SOURCES} )

# 3DX support (3dConnexion devices)
if ( ${OPTION_SUPPORT_3DCONNEXION_DEVICES} )
	file( GLOB 3DX_header_list devices/3dConnexion/*.h )
	file( GLOB 3DX_source_list devices/3dConnexion/*.cpp )
	list( APPEND header_list ${3DX_header_list} )
	list( APPEND source_list ${3DX_source_list} )
endif()	

file( GLOB ui_list ui_templates/*.ui )
file( GLOB qrc_list *.qrc )
#file( GLOB rc_list *.rc )
file( GLOB txt_list TODO.txt bin_other/history.txt )

if ( USE_QT5 )
	qt5_wrap_ui( generated_ui_list ${ui_list} )
	qt5_add_resources( generated_qrc_list ${qrc_list} )
else()
	# find Qt mocable files
	find_mocable_files( mocable_list ${header_list} )
	qt4_wrap_cpp( moc_list ${mocable_list} )
	qt4_wrap_ui( generated_ui_list ${ui_list} )
	qt4_add_resources( generated_qrc_list ${qrc_list} )
endif()

# App icon with MSVC
if( MSVC )
	set( rc_list images/icon/cc_icon.rc )
endif()

if( MSVC )
	#to get rid of the (system) console
	add_executable( ${PROJECT_NAME} WIN32 ${header_list} ${source_list} ${moc_list} ${generated_ui_list} ${generated_qrc_list} ${rc_list} ${resource_list} ${txt_list} )
elseif( APPLE )
	add_executable( ${PROJECT_NAME} MACOSX_BUNDLE ${header_list} ${source_list} ${moc_list} ${generated_ui_list} ${generated_qrc_list} ${resource_list} ${txt_list} )
else()
	add_executable( ${PROJECT_NAME} ${header_list} ${source_list} ${moc_list} ${generated_ui_list} ${generated_qrc_list} ${rc_list} ${resource_list} ${txt_list} )
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
	qt5_use_modules(${PROJECT_NAME} Core Gui Widgets OpenGL PrintSupport)
endif()

# contrib. libraries support
target_link_contrib( ${PROJECT_NAME} ${CLOUDCOMPARE_DEST_FOLDER} )

# 3dConnexion devices support
target_link_3DXWARE( ${PROJECT_NAME} )

# Default preprocessors
set_default_cc_preproc( ${PROJECT_NAME} )

# Add custom preprocessor definitions
set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS USE_GLEW GLEW_STATIC )
if( WIN32 )
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS CC_USE_AS_DLL QCC_DB_USE_AS_DLL QCC_IO_USE_AS_DLL )
	SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LINK_FLAGS " /MANIFEST:NO")
endif()

#set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS_RELEASE XXX) #nothing right now
#set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS_DEBUG XXX) #nothing right now

# App icon with Code::Blocks/MinGW
if( WIN32 )
	if( MINGW )
		add_custom_command( TARGET ${PROJECT_NAME} PRE_BUILD COMMAND windres -i ${CMAKE_CURRENT_SOURCE_DIR}/images/icon/cc_icon.rc --input-format=rc -o ${CMAKE_CURRENT_BINARY_DIR}/cc_icon.res -O coff )
	endif()
endif()

# install program
install_ext( TARGETS ${PROJECT_NAME} ${CLOUDCOMPARE_DEST_FOLDER} )

# Auxiliary files
set( auxFiles bin_other/history.txt bin_other/license.txt bin_other/global_shift_list_template.txt )

if( WIN32 )
	# Export Qt dlls
	install_Qt_Dlls( ${CLOUDCOMPARE_DEST_FOLDER} )
	install_Qt_ImageFormats( ${CLOUDCOMPARE_DEST_FOLDER} )
	install_Qt5_plugins( ${CLOUDCOMPARE_DEST_FOLDER} )

	# Additional auxiliary file(s)
	list( APPEND auxFiles bin_other/start.bat )
endif()

# Install auxiliary files
foreach( filename ${auxFiles} )
	install_ext( FILES ${filename} ${CLOUDCOMPARE_DEST_FOLDER} )
endforeach()

# in order to ensure that everything else is installed first, put the Mac bundling in its own subdirectory
if( APPLE )
   set_property( TARGET ${PROJECT_NAME} PROPERTY MACOSX_BUNDLE_INFO_PLIST ${CMAKE_CURRENT_SOURCE_DIR}/Mac/CloudCompare.plist )

   set( MACOSX_BUNDLE_ICON_FILE cc_icon.icns ) 
   set( MACOSX_BUNDLE_SHORT_VERSION_STRING "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}" )
   set( MACOSX_BUNDLE_LONG_VERSION_STRING "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}" )
   set( MACOSX_BUNDLE_BUNDLE_VERSION "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}" )

   add_subdirectory( Mac )
endif()

# Export common shader files to all install destinations
if( APPLE )
   install( FILES ${CC_FBO_LIB_SOURCE_DIR}/bilateral/bilateral.frag DESTINATION ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Shaders )
   install( FILES ${CC_FBO_LIB_SOURCE_DIR}/bilateral/bilateral.vert DESTINATION ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Shaders )
   install( FILES ${CMAKE_CURRENT_SOURCE_DIR}/shaders/ColorRamp/color_ramp.frag DESTINATION ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Shaders/ColorRamp )
   #install( FILES ${CMAKE_CURRENT_SOURCE_DIR}/shaders/Rendering/rendering.frag DESTINATION ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Shaders/Rendering )
   #install( FILES ${CMAKE_CURRENT_SOURCE_DIR}/shaders/Rendering/rendering.vert DESTINATION ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Shaders/Rendering )
else()
   install_ext( FILES ${CC_FBO_LIB_SOURCE_DIR}/bilateral/bilateral.frag ${CLOUDCOMPARE_DEST_FOLDER} /shaders )
   install_ext( FILES ${CC_FBO_LIB_SOURCE_DIR}/bilateral/bilateral.vert ${CLOUDCOMPARE_DEST_FOLDER} /shaders )
   install_ext( FILES ${CMAKE_CURRENT_SOURCE_DIR}/shaders/ColorRamp/color_ramp.frag ${CLOUDCOMPARE_DEST_FOLDER} /shaders/ColorRamp )
   #install_ext( FILES ${CMAKE_CURRENT_SOURCE_DIR}/shaders/Rendering/rendering.frag ${CLOUDCOMPARE_DEST_FOLDER} /shaders/Rendering )
   #install_ext( FILES ${CMAKE_CURRENT_SOURCE_DIR}/shaders/Rendering/rendering.vert ${CLOUDCOMPARE_DEST_FOLDER} /shaders/Rendering )
endif()
