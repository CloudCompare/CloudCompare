project( CloudCompare VERSION 2.14.0 )

include( CMakePolicies NO_POLICY_SCOPE )

option( CC_USE_EIGEN
	"Compile CloudCompare with Eigen (for registration)"
	OFF
)

if ( CC_USE_EIGEN )
	set ( EIGEN_ROOT_DIR "" CACHE PATH "Eigen root (contains the Eigen directory)" )
	if ( NOT EIGEN_ROOT_DIR )
		message( SEND_ERROR "No Eigen root directory specified (EIGEN_ROOT_DIR)" )
	endif()
endif()

include_directories( ${CloudComparePlugins_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/db_tree )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/ui_templates )
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

file( GLOB header_list *.h  )
file( GLOB source_list *.cpp )

file( GLOB ui_list ui_templates/*.ui )
file( GLOB qrc_list *.qrc )

file( GLOB txt_list TODO.txt ${CloudCompareProjects_SOURCE_DIR}/CHANGELOG.md )

add_subdirectory( db_tree )
add_subdirectory( pluginManager )

add_subdirectory( extern/QCustomPlot EXCLUDE_FROM_ALL )

qt_add_resources( generated_qrc_list ${qrc_list} )

if( MSVC )
	# App icon with MSVC
	set( msvc_icon images/icon/cc_icon.rc )

	qt_add_executable( ${PROJECT_NAME} WIN32 ${header_list} ${source_list} ${generated_qrc_list} ${msvc_icon} ${txt_list} ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/windows/qt5.natvis )
elseif( APPLE )
	qt_add_executable( ${PROJECT_NAME} MACOSX_BUNDLE ${header_list} ${source_list} ${generated_qrc_list} ${txt_list} )
else()
	qt_add_executable( ${PROJECT_NAME} ${header_list} ${source_list} ${generated_qrc_list} ${txt_list} )
endif()

qt_add_ui( ${PROJECT_NAME} SOURCES ${ui_list} )

target_link_libraries( ${PROJECT_NAME}
	PRIVATE
		CCAppCommon
		QCustomPlot
		Qt6::PrintSupport
)

# OpenMP
if( OpenMP_CXX_FOUND )
	target_link_libraries( ${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX )
endif()

# Eigen
if ( CC_USE_EIGEN )
	target_include_directories( ${PROJECT_NAME}
		PUBLIC
			${EIGEN_ROOT_DIR}
	)

	target_compile_definitions( ${PROJECT_NAME}
		PUBLIC
			CC_USES_EIGEN
	)
endif()

# Add custom preprocessor definitions
if( WIN32 )
	if (MSVC)
		target_link_options( ${PROJECT_NAME} PRIVATE "/MANIFEST:NO" )
	endif()
endif()

# App icon with Code::Blocks/MinGW
if( WIN32 )
	if( MINGW )
		if (NOT CMAKE_RC_COMPILER)
			set( CMAKE_RC_COMPILER windres )
		endif()
		add_custom_command( TARGET ${PROJECT_NAME} PRE_BUILD COMMAND ${CMAKE_RC_COMPILER} -i ${CMAKE_CURRENT_SOURCE_DIR}/images/icon/cc_icon.rc --input-format=rc -o ${CMAKE_CURRENT_BINARY_DIR}/cc_icon.res -O coff )
	endif()
endif()

install( TARGETS
	${PROJECT_NAME}
		BUNDLE  DESTINATION ${CLOUDCOMPARE_DEST_FOLDER}/ # On macOS the bundle will be in $install_dir/$app_name/$app_name.app (the name of the bundle is defined by the qt_add_executable command)
		RUNTIME DESTINATION ${CLOUDCOMPARE_DEST_FOLDER}/
)

# Auxiliary files
set( auxFiles  ${CloudCompareProjects_SOURCE_DIR}/CHANGELOG.md bin_other/license.txt bin_other/global_shift_list_template.txt )

if( WIN32 )
	# Additional auxiliary file(s)
	file( GLOB batch_list bin_other/*.bat )
	list( APPEND auxFiles ${batch_list} )
endif()

# Install auxiliary files
if( WIN32 OR APPLE )
	foreach( filename ${auxFiles} )
		install( FILES ${filename} DESTINATION ${CLOUDCOMPARE_DEST_FOLDER} )
	endforeach()
else( UNIX )
	foreach( filename ${auxFiles} )
		install( FILES ${filename} DESTINATION share/cloudcompare  )
	endforeach()
endif()

# Put together our Application bundle on macOS
if( APPLE )
	add_subdirectory( Mac )
endif()

# Translation
add_subdirectory( translations )

## Shaders
# Copy shader dirs into our shadow build directory
file( COPY ${CC_FBO_LIB_SOURCE_DIR}/shaders DESTINATION "${CMAKE_BINARY_DIR}" )
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/shaders DESTINATION "${CMAKE_BINARY_DIR}" )

# Export common shader files to all install destinations
if( APPLE )
	install( FILES ${CC_FBO_LIB_SOURCE_DIR}/shaders/Bilateral/bilateral.frag DESTINATION ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Shaders/Bilateral )
	install( FILES ${CC_FBO_LIB_SOURCE_DIR}/shaders/Bilateral/bilateral.vert DESTINATION ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Shaders/Bilateral )
	install( FILES ${CMAKE_CURRENT_SOURCE_DIR}/shaders/ColorRamp/color_ramp.frag DESTINATION ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Shaders/ColorRamp )
	install( FILES ${CMAKE_CURRENT_SOURCE_DIR}/shaders/DrawNormals/DrawNormals.vs DESTINATION ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Shaders/DrawNormals )
	install( FILES ${CMAKE_CURRENT_SOURCE_DIR}/shaders/DrawNormals/DrawNormals.gs DESTINATION ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Shaders/DrawNormals )
	install( FILES ${CMAKE_CURRENT_SOURCE_DIR}/shaders/DrawNormals/DrawNormals.fs DESTINATION ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Shaders/DrawNormals )
elseif( UNIX )
	install( FILES ${CC_FBO_LIB_SOURCE_DIR}/shaders/Bilateral/bilateral.frag DESTINATION share/cloudcompare/shaders/Bilateral )
	install( FILES ${CC_FBO_LIB_SOURCE_DIR}/shaders/Bilateral/bilateral.vert DESTINATION share/cloudcompare/shaders/Bilateral )
	install( FILES ${CMAKE_CURRENT_SOURCE_DIR}/shaders/ColorRamp/color_ramp.frag DESTINATION share/cloudcompare/shaders/ColorRamp )
	install( FILES ${CMAKE_CURRENT_SOURCE_DIR}/shaders/DrawNormals/DrawNormals.vs DESTINATION share/cloudcompare/shaders/DrawNormals )
	install( FILES ${CMAKE_CURRENT_SOURCE_DIR}/shaders/DrawNormals/DrawNormals.gs DESTINATION share/cloudcompare/shaders/DrawNormals )
	install( FILES ${CMAKE_CURRENT_SOURCE_DIR}/shaders/DrawNormals/DrawNormals.fs DESTINATION share/cloudcompare/shaders/DrawNormals )
else()
	install( FILES ${CC_FBO_LIB_SOURCE_DIR}/shaders/Bilateral/bilateral.frag DESTINATION ${CLOUDCOMPARE_DEST_FOLDER}/shaders/Bilateral )
	install( FILES ${CC_FBO_LIB_SOURCE_DIR}/shaders/Bilateral/bilateral.vert DESTINATION ${CLOUDCOMPARE_DEST_FOLDER}/shaders/Bilateral )
	install( FILES ${CMAKE_CURRENT_SOURCE_DIR}/shaders/ColorRamp/color_ramp.frag DESTINATION ${CLOUDCOMPARE_DEST_FOLDER}/shaders/ColorRamp )
	install( FILES ${CMAKE_CURRENT_SOURCE_DIR}/shaders/DrawNormals/DrawNormals.vs DESTINATION ${CLOUDCOMPARE_DEST_FOLDER}/shaders/DrawNormals )
	install( FILES ${CMAKE_CURRENT_SOURCE_DIR}/shaders/DrawNormals/DrawNormals.gs DESTINATION ${CLOUDCOMPARE_DEST_FOLDER}/shaders/DrawNormals )
	install( FILES ${CMAKE_CURRENT_SOURCE_DIR}/shaders/DrawNormals/DrawNormals.fs DESTINATION ${CLOUDCOMPARE_DEST_FOLDER}/shaders/DrawNormals )
endif()

# Install plugins and shaders in the correct folder for each platform
if( APPLE )
	InstallPlugins(
		DEST_PATH "${CLOUDCOMPARE_MAC_PLUGIN_DIR}"
		SHADER_DEST_PATH "${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Shaders"
	)
elseif( UNIX )
	InstallPlugins(
		DEST_PATH "${CMAKE_INSTALL_LIBDIR}/cloudcompare/plugins"
		SHADER_DEST_PATH share/cloudcompare/shaders
	)
else()
	InstallPlugins(
		DEST_PATH "${CLOUDCOMPARE_DEST_FOLDER}"
		DEST_FOLDER "plugins"
		SHADER_DEST_PATH "${CLOUDCOMPARE_DEST_FOLDER}"
		SHADER_DEST_FOLDER "shaders"
	)
endif()

if( APPLE )
	# We use macdeployqt to deploy runtime dependencies.
	# Signature could be needed
	qt_generate_deploy_script(
		TARGET ${PROJECT_NAME}
		OUTPUT_SCRIPT deploy_cc
		CONTENT "
			qt_deploy_runtime_dependencies(
				EXECUTABLE \"${CLOUDCOMPARE_DEST_FOLDER}/${PROJECT_NAME}.app\"
				NO_APP_STORE_COMPLIANCE
		)")

	install( SCRIPT ${deploy_cc} )
elseif( WIN32 )
	# We are looking for Qt dependencies inside additional libraries
	# it notably find QtConcurrent (if needed by CCCoreLib) and QOpenGLWidgets
	set( ADDITIONAL_LIB_LINES "" )
	foreach( lib IN LISTS CC_SHARED_LIB_FILENAMES )
		string( APPEND ADDITIONAL_LIB_LINES "ADDITIONAL_LIBRARIES \"${CLOUDCOMPARE_DEST_FOLDER}/${lib}\"\n" )
	endforeach()

	# We do the same for plugins
	foreach( plugin IN LISTS installed_plugin_targets )
		string( APPEND ADDITIONAL_LIB_LINES "ADDITIONAL_LIBRARIES \"${CLOUDCOMPARE_DEST_FOLDER}/plugins/$<TARGET_FILE_NAME:${plugin}>\"\n" )
	endforeach()

	qt_generate_deploy_script(
		TARGET ${PROJECT_NAME}
		OUTPUT_SCRIPT deploy_cc
		CONTENT "
			qt_deploy_runtime_dependencies(
				EXECUTABLE \"${CLOUDCOMPARE_DEST_FOLDER}/${PROJECT_NAME}.exe\"
				${ADDITIONAL_LIB_LINES}
				BIN_DIR \"${CLOUDCOMPARE_DEST_FOLDER}/\"
				LIB_DIR \"${CLOUDCOMPARE_DEST_FOLDER}/\"
				PLUGINS_DIR \"${CLOUDCOMPARE_DEST_FOLDER}/\"
				NO_COMPILER_RUNTIME
				NO_TRANSLATIONS
				DEPLOY_TOOL_OPTIONS --no-quick-import --no-system-dxc-compiler --no-system-d3d-compiler --verbose=1
			)") # We inibit translation deployment because we already copy all needed files via the cmake script in the translation subdirectory.

	install( SCRIPT ${deploy_cc} )
endif()

# Add a 'fake' custom target to force the recompilation of ccApplication.ccp in order to
# always get an up-to-date version number (based on the current date)
add_custom_target(CC_UPDATE_VERSION ALL COMMAND ${CMAKE_COMMAND} -E touch_nocreate ${CMAKE_CURRENT_SOURCE_DIR}/ccApplication.cpp)
