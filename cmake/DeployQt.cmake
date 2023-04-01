find_package( Qt5
	COMPONENTS
		Core
	REQUIRED
)

get_target_property( qmake_location Qt5::qmake IMPORTED_LOCATION )
get_filename_component( qt5_bin_dir ${qmake_location} DIRECTORY )

if ( APPLE )
	find_program( mac_deploy_qt macdeployqt HINTS "${qt5_bin_dir}" )

	if( NOT EXISTS "${mac_deploy_qt}" )
		message( FATAL_ERROR "macdeployqt not found in ${qt5_bin_dir}" )
	endif()
elseif( WIN32 )
	find_program( win_deploy_qt windeployqt HINTS "${qt5_bin_dir}" )

	if( NOT EXISTS "${win_deploy_qt}" )
		message( FATAL_ERROR "windeployqt not found in ${qt5_bin_dir}" )
	endif()
endif()

function( DeployQt )
	if ( NOT APPLE AND NOT WIN32 )
		return()
	endif()

	cmake_parse_arguments(
			DEPLOY_QT
			""
			"TARGET;DEPLOY_PATH"
			""
			${ARGN}
	)

	if( NOT DEPLOY_QT_DEPLOY_PATH )
		message( FATAL_ERROR "DeployQt: DEPLOY_PATH not set" )
	endif()

	# For readability
	set( deploy_path "${DEPLOY_QT_DEPLOY_PATH}" )
	
	message( STATUS "Installing ${DEPLOY_QT_TARGET} to ${deploy_path}" )
	
	get_target_property( name ${DEPLOY_QT_TARGET} NAME )
		
	if ( APPLE )
		set( app_name "${name}.app" )
		if (CMAKE_CONFIGURATION_TYPES)
			set(app_path "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/${app_name}")
			set(temp_dir "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/deployqt")
		else ()
			set(app_path "${CMAKE_CURRENT_BINARY_DIR}/${app_name}")
			set(temp_dir "${CMAKE_CURRENT_BINARY_DIR}/deployqt")
		endif ()
		set( temp_app_path "${temp_dir}/$<CONFIG>/${app_name}" )

		add_custom_command(
			TARGET ${DEPLOY_QT_TARGET}
			POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E remove_directory "${temp_dir}"
			COMMAND ${CMAKE_COMMAND} -E make_directory "${temp_dir}"
			COMMAND ${CMAKE_COMMAND} -E copy_directory ${app_path} ${temp_app_path}
			COMMAND "${mac_deploy_qt}"
				${temp_app_path}
				-verbose=1
			VERBATIM
		)

		install(
			DIRECTORY ${temp_app_path}
			DESTINATION "${deploy_path}"
			USE_SOURCE_PERMISSIONS
		)
	elseif( WIN32 )	
		set( app_name "${name}.exe" )
		if( CMAKE_CONFIGURATION_TYPES )
			set( app_path "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/${app_name}" )
			set( temp_dir "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/deployqt" )
		else()
			set( app_path "${CMAKE_CURRENT_BINARY_DIR}/${app_name}" )
			set( temp_dir "${CMAKE_CURRENT_BINARY_DIR}/deployqt" )
		endif()
		set( temp_app_path "${temp_dir}/${app_name}" )

		add_custom_command(
			TARGET ${DEPLOY_QT_TARGET}
			POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E remove_directory "${temp_dir}"
			COMMAND ${CMAKE_COMMAND} -E make_directory "${temp_dir}"
			COMMAND ${CMAKE_COMMAND} -E copy ${app_path} ${temp_app_path}
			COMMAND "${win_deploy_qt}"
				${temp_app_path}
				--no-compiler-runtime
				--no-angle
				--no-opengl-sw
				--no-quick-import
				--no-system-d3d-compiler
				--concurrent
				--verbose=1
				--gamepad
			VERBATIM
		)
	
		if( NOT CMAKE_CONFIGURATION_TYPES )
			install(
				DIRECTORY ${temp_dir}/
				DESTINATION ${deploy_path}
			)
		else()
			install(
				DIRECTORY ${temp_dir}/
				CONFIGURATIONS Debug
				DESTINATION ${deploy_path}_debug
			)
		
			install(
				DIRECTORY ${temp_dir}/
				CONFIGURATIONS Release
				DESTINATION ${deploy_path}
			)
		
			install(
				DIRECTORY ${temp_dir}/
				CONFIGURATIONS RelWithDebInfo
				DESTINATION ${deploy_path}_withDebInfo
			)
		endif()
	endif()
endfunction()

unset( qmake_location )
unset( qt5_bin_dir )
