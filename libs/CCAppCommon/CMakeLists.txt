project( CCAppCommon )

add_library( ${PROJECT_NAME} SHARED )

add_subdirectory( devices )
add_subdirectory( include )
add_subdirectory( src )
add_subdirectory( ui )

target_link_libraries( ${PROJECT_NAME}
	PUBLIC
		CCPluginAPI
	PRIVATE
		CC_FBO_LIB
)

target_compile_definitions( ${PROJECT_NAME} PRIVATE CCAPPCOMMON_LIBRARY_BUILD )

set_target_properties( ${PROJECT_NAME} PROPERTIES
	AUTOUIC ON # FIXME Remove after everything has moved to targets and we can set it globally
	CXX_VISIBILITY_PRESET hidden
)

target_sources(${PROJECT_NAME}
		PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/QDarkStyleSheet/qdarkstyle/dark/darkstyle.qrc
		${CMAKE_CURRENT_LIST_DIR}/QDarkStyleSheet/qdarkstyle/light/lightstyle.qrc
)

# We run find_package here for HIDAPI since it will link to the CCAppCommon
# if we run it only in the 3dConnexion subdir, it fails to create hidapi::hidapi targets in scope
find_package( hidapi 0.13.0 CONFIG QUIET )

if( OPTION_SUPPORT_3DMOUSE_WITH_3DxWARE )
	target_link_3DxWARE( ${PROJECT_NAME} )
endif()

InstallSharedLibrary( TARGET ${PROJECT_NAME} )
