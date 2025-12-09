# InstallSharedLibrary should be called once for each shared library in the libs directory.
# This function installs the shared library in the correct places for each platform.
#
# If INSTALL_DESTINATIONS is not empty, it will install to each of the destinations in the list.
# If INSTALL_DESTINATIONS is empty, this function does nothing.
#
# Arguments:
#	TARGET The name of the library target
function( InstallSharedLibrary )
	if( NOT INSTALL_DESTINATIONS )
		return()
	endif()

	cmake_parse_arguments(
			INSTALL_SHARED_LIB
			""
			"TARGET"
			""
			${ARGN}
	)

	# For readability
	set( shared_lib_target "${INSTALL_SHARED_LIB_TARGET}" )
	message( STATUS "Install shared library: ${shared_lib_target}" )

	if( WIN32 )
		# collect filenames for Qt deployment
		list(APPEND CC_SHARED_LIB_FILENAMES "$<TARGET_FILE_NAME:${shared_lib_target}>" )
		set( CC_SHARED_LIB_FILENAMES ${CC_SHARED_LIB_FILENAMES} CACHE INTERNAL "" FORCE )
	endif()

	foreach( destination ${INSTALL_DESTINATIONS} )
		if( UNIX AND NOT APPLE )
			set( destination ${LINUX_INSTALL_SHARED_DESTINATION} )
		endif()
		_InstallSharedTarget(
				TARGET ${shared_lib_target}
				DEST_PATH ${destination}
		)
	endforeach ()
endfunction()

# InstallFiles should be called to install files that are not targets.
# This function installs the files in the correct places for each platform.
#
# If INSTALL_DESTINATIONS is not empty, it will install to each of the destinations in the list.
# If INSTALL_DESTINATIONS is empty, this function does nothing.
#
# Arguments:
#	FILES The name of the files to install
function( InstallFiles )
	if( NOT INSTALL_DESTINATIONS )
		return()
	endif()

	cmake_parse_arguments(
			INSTALL_FILES
			""
			""
			"FILES"
			${ARGN}
	)

	# For readability
	set( files "${INSTALL_FILES_FILES}" )

	if( NOT files )
		message( WARNING "InstallFiles: no files specified" )
		return()
	endif()

	message( STATUS "Install files: ${files} to ${INSTALL_DESTINATIONS}")

	foreach( destination ${INSTALL_DESTINATIONS} )
		_InstallFiles(
			FILES ${files}
			DEST_PATH ${destination}
		)
	endforeach()
endfunction()

# InstallPlugins should be called once for each application.
# This function installs the plugin types that are requested in TYPES to the path specified by DEST_FOLDER.
# If it is a gl plugin with shaders, install the shaders to SHADER_DEST_FOLDER.
#
# Arguments:
#	DEST_FOLDER The name of the directory to install the plugins in.
#	DEST_PATH Path to DEST_FOLDER - note that on Windows we will modify this depending on CONFIGURATIONS
#	SHADER_DEST_FOLDER The name of the directory to install the shaders for the plugins.
#	SHADER_DEST_PATH Path to SHADER_DEST_FOLDER - note that on Windows we will modify this depending on CONFIGURATIONS
#	TYPES Semicolon-separated list of plugin types to install (valid: gl, io, standard). If not specified, install all.
function( InstallPlugins )
	cmake_parse_arguments(
			INSTALL_PLUGINS
			""
			"DEST_FOLDER;DEST_PATH;SHADER_DEST_FOLDER;SHADER_DEST_PATH"
			"TYPES"
			${ARGN}
	)

	# Check the types we need to install
	set( VALID_TYPES "gl" "io" "standard" )

	# If TYPES was not specified, use all of them
	if( NOT INSTALL_PLUGINS_TYPES )
		set( INSTALL_PLUGINS_TYPES "${VALID_TYPES}" )
	else()
		foreach( type ${INSTALL_PLUGINS_TYPES} )
			if( NOT "${type}" IN_LIST VALID_TYPES )
				list( JOIN VALID_TYPES ", " VALID_TYPES_STR )
				message( FATAL_ERROR "InstallPlugins: Did not find proper TYPES. Valid values are: ${VALID_TYPES_STR}" )
			endif()
		endforeach()
	endif()

	message( STATUS "Install plugins" )
	message( STATUS " Types: ${INSTALL_PLUGINS_TYPES}" )

	# Check our destination path is valid
	if( NOT INSTALL_PLUGINS_DEST_PATH )
		message( FATAL_ERROR "InstallPlugins: DEST_PATH not specified" )
	endif()

	message( STATUS " Destination: ${INSTALL_PLUGINS_DEST_PATH}/${INSTALL_PLUGINS_DEST_FOLDER}" )

	# If we have gl plugins, check that our shader destination folder is valid
	if( "gl" IN_LIST INSTALL_PLUGINS_TYPES )
		if( NOT INSTALL_PLUGINS_SHADER_DEST_PATH )
			message( FATAL_ERROR "InstallPlugins: SHADER_DEST_PATH not specified" )
		endif()

		message( STATUS " Shader Destination: ${INSTALL_PLUGINS_SHADER_DEST_PATH}/${INSTALL_PLUGINS_SHADER_DEST_FOLDER}" )
	endif()

	# Make CloudCompare/ccViewer depend on the plugins
	# so that when building CloudCompare/ccViewer the plugins also get built
	# instead of waiting for the `install` target to be ran for the plugins to get built
	if ( CC_PLUGIN_TARGET_LIST )
		add_dependencies( ${PROJECT_NAME} ${CC_PLUGIN_TARGET_LIST} )
	endif()

	# Install the requested plugins in the DEST_FOLDER
 	set( installed_plugins "" )
	foreach( plugin_target ${CC_PLUGIN_TARGET_LIST} )
		get_target_property( plugin_type ${plugin_target} PLUGIN_TYPE )

		if( "${plugin_type}" IN_LIST INSTALL_PLUGINS_TYPES )
			message( STATUS " Install ${plugin_target} (${plugin_type})" )
			list( APPEND installed_plugins ${plugin_target} )
			_InstallSharedTarget(
				TARGET ${plugin_target}
				DEST_PATH ${INSTALL_PLUGINS_DEST_PATH}
				DEST_FOLDER ${INSTALL_PLUGINS_DEST_FOLDER}
			)

			if( "${plugin_type}" STREQUAL "gl" )
				get_target_property( SHADER_FOLDER_NAME ${plugin_target} SHADER_FOLDER_NAME )
				get_target_property( SHADER_FOLDER_PATH ${plugin_target} SHADER_FOLDER_PATH )

				if( EXISTS "${SHADER_FOLDER_PATH}" )
					message( STATUS "  + shader: ${SHADER_FOLDER_NAME} (${SHADER_FOLDER_PATH})" )

					get_target_property( shader_files ${plugin_target} SOURCES )
					list( FILTER shader_files INCLUDE REGEX ".*\.vert|frag" )

					_InstallFiles(
						FILES ${shader_files}
						DEST_PATH ${INSTALL_PLUGINS_SHADER_DEST_PATH}
						DEST_FOLDER ${INSTALL_PLUGINS_SHADER_DEST_FOLDER}/${SHADER_FOLDER_NAME}
					)
				endif()
			endif()
		endif()
	endforeach()
	set( installed_plugin_targets "${installed_plugins}" PARENT_SCOPE)
endfunction()

# _InstallSharedTarget should only be called by one of the functions above.
# Arguments:
#	DEST_FOLDER The name of the directory to install the shared lib in.
#	DEST_PATH Path to DEST_FOLDER - note that on Windows we will modify this depending on CONFIGURATIONS
#	TARGET The name of the shared lib target
function( _InstallSharedTarget )
	cmake_parse_arguments(
			INSTALL_SHARED_TARGET
			""
			"DEST_FOLDER;DEST_PATH;TARGET"
			""
			${ARGN}
	)

	# For readability
	set( shared_target "${INSTALL_SHARED_TARGET_TARGET}" )
	set( full_path "${INSTALL_SHARED_TARGET_DEST_PATH}/${INSTALL_SHARED_TARGET_DEST_FOLDER}" )
	install(
		TARGETS ${shared_target}
		RUNTIME DESTINATION ${full_path}
		LIBRARY DESTINATION ${full_path}
		COMPONENT Runtime
	)
endfunction()

# _InstallFiles should only be called by one of the functions above.
#
# Arguments:
#	DEST_FOLDER The name of the directory to install the files in.
#	DEST_PATH Path to DEST_FOLDER - note that on Windows we will modify this depending on CONFIGURATIONS
#	FILES The name of the files to install
function( _InstallFiles )
	cmake_parse_arguments(
			INSTALL_FILES
			""
			"DEST_FOLDER;DEST_PATH"
			"FILES"
			${ARGN}
	)

	# For readability
	set( files "${INSTALL_FILES_FILES}" )
	set( full_path "${INSTALL_FILES_DEST_PATH}/${INSTALL_FILES_DEST_FOLDER}" )
	cmake_path(SET full_path NORMALIZE "${INSTALL_FILES_DEST_PATH}/${INSTALL_FILES_DEST_FOLDER}" )

	install(
		FILES ${files}
		DESTINATION "${full_path}"
	)
endfunction()
