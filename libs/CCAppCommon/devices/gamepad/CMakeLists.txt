
option( OPTION_SUPPORT_GAMEPADS "Build with gamepad support (requires Qt 5.7+)" OFF )
if( OPTION_SUPPORT_GAMEPADS )
	target_sources( ${PROJECT_NAME}
		PRIVATE
		    ${CMAKE_CURRENT_LIST_DIR}/ccGamepadManager.h
			${CMAKE_CURRENT_LIST_DIR}/ccGamepadManager.cpp
			${CMAKE_CURRENT_LIST_DIR}/GamepadInput.h
			${CMAKE_CURRENT_LIST_DIR}/GamepadInput.cpp
	)

	target_include_directories( ${PROJECT_NAME}
		PUBLIC
			${CMAKE_CURRENT_LIST_DIR}
	)

    target_compile_definitions( ${PROJECT_NAME} PUBLIC CC_GAMEPAD_SUPPORT )
endif()
