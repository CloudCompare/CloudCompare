project( ccViewer VERSION 1.38.0 )

include( CMakePolicies NO_POLICY_SCOPE )

include_directories( ${CloudComparePlugins_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/../qCC )
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

file( GLOB header_list *.h )
file( GLOB source_list *.cpp )

file( GLOB ui_list ui_templates/*.ui )
file( GLOB qrc_list ../qCC/*.qrc )

qt_add_resources( generated_qrc_list ${qrc_list} )

if( MSVC )
	# App icon with MSVC
	set( msvc_icon images/icon/cc_viewer_icon.rc )

	# to get rid of the (system) console
	qt_add_executable( ${PROJECT_NAME} WIN32 ${header_list} ${source_list} ${generated_qrc_list} ${msvc_icon} ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/windows/qt5.natvis )
elseif( APPLE )
	qt_add_executable( ${PROJECT_NAME} MACOSX_BUNDLE ${header_list} ${source_list} ${generated_qrc_list} )
else()
	qt_add_executable( ${PROJECT_NAME} ${header_list} ${source_list} ${generated_qrc_list} )
endif()

qt_add_ui(${PROJECT_NAME} SOURCES ${ui_list})

target_link_libraries( ${PROJECT_NAME}
	PUBLIC
		CCPluginStub
)

target_link_libraries( ${PROJECT_NAME}
	PRIVATE
		CCAppCommon
)

# Add custom preprocessor definitions
target_compile_definitions( ${PROJECT_NAME} PUBLIC CC_VIEWER )

if (WIN32)
	if (MSVC)
		set_target_properties( ${PROJECT_NAME} PROPERTIES LINK_FLAGS " /MANIFEST:NO" )
	endif()
endif()

# App icon with Code::Blocks/MinGW
if( WIN32 )
	if( MINGW )
		add_custom_command( TARGET ${PROJECT_NAME} PRE_BUILD COMMAND ${CMAKE_RC_COMPILER} -i ${CMAKE_CURRENT_SOURCE_DIR}/images/icon/cc_viewer_icon.rc --input-format=rc -o ${CMAKE_CURRENT_BINARY_DIR}/ccViewer.res -O coff )
	endif()
endif()

install(TARGETS
	${PROJECT_NAME}
		BUNDLE  DESTINATION ${PROJECT_NAME}/ # On macOS the bundle will be in $install_dir/$app_name/$app_name.app (the name of the bundle is defined by the qt_add_executable command)
		RUNTIME DESTINATION ${CCVIEWER_DEST_FOLDER}/
)

# Auxiliary files
set( auxFiles bin_other/license.txt )

if( WIN32 )
	# Additional auxiliary file(s)
	list( APPEND auxFiles bin_other/start.bat )
endif()

# Install auxiliary files
if(APPLE OR WIN32) # For Linux it's already installed in by qCC
	foreach( filename ${auxFiles} )
		install( FILES ${filename} DESTINATION ${CCVIEWER_DEST_FOLDER} )
	endforeach()
endif()

# Put together our Application bundle on macOS
if( APPLE )
	add_subdirectory( Mac )
endif( APPLE )

# Export common shader files to all install destinations
if( WIN32 ) # For Linux it's already installed in by qCC
	install( FILES ${CMAKE_CURRENT_SOURCE_DIR}/../qCC/shaders/ColorRamp/color_ramp.frag DESTINATION ${CCVIEWER_DEST_FOLDER}/shaders/ColorRamp )
endif()

# Install plugins & shaders in the correct folder for each platform
if( APPLE )
	InstallPlugins(
		TYPES "gl;io"
		DEST_PATH "${CCVIEWER_MAC_PLUGIN_DIR}"
		SHADER_DEST_PATH "${CCVIEWER_MAC_BASE_DIR}/Contents/Shaders"
	)
elseif( UNIX )
	InstallPlugins(
		TYPES "gl;io"
		DEST_PATH "${CMAKE_INSTALL_LIBDIR}/cloudcompare/plugins"
		SHADER_DEST_PATH share/cloudcompare/shaders
	)
else()
	InstallPlugins(
		TYPES "gl;io"
		DEST_PATH "${CCVIEWER_DEST_FOLDER}"
		DEST_FOLDER "plugins"
		SHADER_DEST_PATH "${CCVIEWER_DEST_FOLDER}"
		SHADER_DEST_FOLDER "shaders"
)
endif()

# deploy scripts
if( APPLE )
	# Code signature could be needed at this point
	qt_generate_deploy_script(
		TARGET ${PROJECT_NAME}
		OUTPUT_SCRIPT deploy_ccViewer
		CONTENT "
			qt_deploy_runtime_dependencies(
				EXECUTABLE \"${PROJECT_NAME}/${PROJECT_NAME}.app\"
			)"
	)

	install(SCRIPT ${deploy_ccViewer})
elseif( WIN32 )
	set(ADDITIONAL_LIB_LINES "")
	foreach(lib IN LISTS CC_SHARED_LIB_FILENAMES)
		string(APPEND ADDITIONAL_LIB_LINES "ADDITIONAL_LIBRARIES \"${CCVIEWER_DEST_FOLDER}/${lib}\"\n")
	endforeach()

	qt_generate_deploy_script(
		TARGET ${PROJECT_NAME}
		OUTPUT_SCRIPT deploy_ccViewer
		CONTENT "
		qt_deploy_runtime_dependencies(
			EXECUTABLE \"${CCVIEWER_DEST_FOLDER}/${PROJECT_NAME}.exe\"
			${ADDITIONAL_LIB_LINES}
			BIN_DIR \"${CCVIEWER_DEST_FOLDER}/\"
			LIB_DIR \"${CCVIEWER_DEST_FOLDER}/\"
			PLUGINS_DIR \"${CCVIEWER_DEST_FOLDER}/\"
			NO_COMPILER_RUNTIME
			DEPLOY_TOOL_OPTIONS --no-quick-import --no-system-dxc-compiler --no-system-d3d-compiler --verbose=1 --translationdir ${CCVIEWER_DEST_FOLDER}/translations
		)"
	)
	install(SCRIPT ${deploy_ccViewer})
endif()

# Add a 'fake' custom target to force the recompilation of ccApplication.ccp in order to
# always get an up-to-date version number (based on the current date)
add_custom_target(CC_VIEWER_UPDATE_VERSION ALL COMMAND ${CMAKE_COMMAND} -E touch_nocreate ${CMAKE_CURRENT_SOURCE_DIR}/ccViewerApplication.cpp)
