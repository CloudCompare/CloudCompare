project( CCAppCommon )

add_library( ${PROJECT_NAME} SHARED )

add_subdirectory( devices )
add_subdirectory( include )
add_subdirectory( src )
add_subdirectory( ui )

target_link_libraries( ${PROJECT_NAME}
	CCPluginAPI
)

target_compile_definitions( ${PROJECT_NAME} PRIVATE CCAPPCOMMON_LIBRARY_BUILD )

set_target_properties( ${PROJECT_NAME} PROPERTIES
	AUTOUIC ON # FIXME Remove after everything has moved to targets and we can set it globally
	CXX_VISIBILITY_PRESET hidden
)

if( OPTION_SUPPORT_GAMEPADS )
	find_package( Qt5
		COMPONENTS
			Gamepad
		REQUIRED
	)

	target_link_libraries( ${PROJECT_NAME} Qt5::Gamepad )
endif()

InstallSharedLibrary( TARGET ${PROJECT_NAME} )
