# ------------------------------------------------------------------------------
# Gamepad support for CloudCompare
# ------------------------------------------------------------------------------

OPTION( OPTION_SUPPORT_GAMEPADS "Build with gamepad support (requires Qt 5.7+)" OFF )
if( ${OPTION_SUPPORT_GAMEPADS} )

	#TODO: make sure that Qt version is at least 5.7 ;)
	find_package(Qt5Gamepad REQUIRED)
	
	include_directories( ${Qt5Gamepad_INCLUDE_DIRS} )


endif()

# link project with GAMEPADS libraries (support actually ;)
function( target_link_GAMEPADS ) # 1 argument: ARGV0 = project name

	if( ${OPTION_SUPPORT_GAMEPADS} )
	
		target_link_libraries(${PROJECT_NAME} Qt5::Gamepad)
		
		set_property( TARGET ${ARGV0} APPEND PROPERTY COMPILE_DEFINITIONS CC_GAMEPADS_SUPPORT )

	endif()

endfunction()
