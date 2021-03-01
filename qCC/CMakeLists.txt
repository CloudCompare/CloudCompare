project( CloudCompare VERSION 2.12.0 )

include( CMakePolicies NO_POLICY_SCOPE )

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

qt5_wrap_ui( generated_ui_list ${ui_list} )
qt5_add_resources( generated_qrc_list ${qrc_list} )

if( MSVC )
	# App icon with MSVC
	set( rc_list images/icon/cc_icon.rc )

	#to get rid of the (system) console
	add_executable( ${PROJECT_NAME} WIN32 ${header_list} ${source_list} ${generated_ui_list} ${generated_qrc_list} ${rc_list} ${txt_list} ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/windows/qt5.natvis )
elseif( APPLE )
	add_executable( ${PROJECT_NAME} MACOSX_BUNDLE ${header_list} ${source_list} ${generated_ui_list} ${generated_qrc_list} ${txt_list} )
else()
	add_executable( ${PROJECT_NAME} ${header_list} ${source_list} ${generated_ui_list} ${generated_qrc_list} ${rc_list} ${txt_list} )
endif()

DeployQt(
	TARGET
		${PROJECT_NAME}
	DEPLOY_PATH
		${PROJECT_NAME}
)

target_link_libraries( ${PROJECT_NAME}
    CCAppCommon
    QCustomPlot
    Qt5::PrintSupport
)

if( CCCORELIB_USE_QT_CONCURRENT )
	# We want *deployqt to handle this dependency, so instead of just copying the file
	# link to the target.
	target_link_libraries( ${PROJECT_NAME}
		Qt5::Concurrent
	)
endif()

# contrib. libraries support
if( APPLE )
	target_link_contrib( ${PROJECT_NAME} ${CLOUDCOMPARE_MAC_FRAMEWORK_DIR} )
elseif( UNIX )
	target_link_contrib( ${PROJECT_NAME} lib/cloudcompare )
else()
	target_link_contrib( ${PROJECT_NAME} ${CLOUDCOMPARE_DEST_FOLDER} )
endif()

# Add custom preprocessor definitions
if( WIN32 )
	if (MSVC)
		set_target_properties( ${PROJECT_NAME} PROPERTIES LINK_FLAGS " /MANIFEST:NO" )
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

# install program
install_ext( TARGETS ${PROJECT_NAME} ${CLOUDCOMPARE_DEST_FOLDER} "" )

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
		install_ext( FILES ${filename} ${CLOUDCOMPARE_DEST_FOLDER} "" )
	endforeach()
else( UNIX )
	foreach( filename ${auxFiles} )
		install_ext( FILES ${filename} share/cloudcompare "" )
	endforeach()
endif()

# Translation
add_subdirectory( translations )

# Put together our Application bundle on macOS
if( APPLE )    
    add_subdirectory( Mac )
endif()

## Shaders
# Copy shader dirs into our shadow build directory
file( COPY ${CC_FBO_LIB_SOURCE_DIR}/shaders DESTINATION "${CMAKE_BINARY_DIR}" )
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/shaders DESTINATION "${CMAKE_BINARY_DIR}" )

# Export common shader files to all install destinations
if( APPLE )	
	install( FILES ${CC_FBO_LIB_SOURCE_DIR}/shaders/Bilateral/bilateral.frag DESTINATION ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Shaders/Bilateral )
	install( FILES ${CC_FBO_LIB_SOURCE_DIR}/shaders/Bilateral/bilateral.vert DESTINATION ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Shaders/Bilateral )
	install( FILES ${CMAKE_CURRENT_SOURCE_DIR}/shaders/ColorRamp/color_ramp.frag DESTINATION ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Shaders/ColorRamp )
elseif( UNIX )
	install( FILES ${CC_FBO_LIB_SOURCE_DIR}/shaders/Bilateral/bilateral.frag DESTINATION share/cloudcompare/shaders/Bilateral )
	install( FILES ${CC_FBO_LIB_SOURCE_DIR}/shaders/Bilateral/bilateral.vert DESTINATION share/cloudcompare/shaders/Bilateral )
	install( FILES ${CMAKE_CURRENT_SOURCE_DIR}/shaders/ColorRamp/color_ramp.frag DESTINATION share/cloudcompare/shaders/ColorRamp )
else()
	install_ext( FILES ${CC_FBO_LIB_SOURCE_DIR}/shaders/Bilateral/bilateral.frag ${CLOUDCOMPARE_DEST_FOLDER} /shaders/Bilateral )
	install_ext( FILES ${CC_FBO_LIB_SOURCE_DIR}/shaders/Bilateral/bilateral.vert ${CLOUDCOMPARE_DEST_FOLDER} /shaders/Bilateral )
	install_ext( FILES ${CMAKE_CURRENT_SOURCE_DIR}/shaders/ColorRamp/color_ramp.frag ${CLOUDCOMPARE_DEST_FOLDER} /shaders/ColorRamp )
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
