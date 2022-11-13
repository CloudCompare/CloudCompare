# InstallSharedLibrary should be called once for each shared library in the libs directory.
# This function installs the shared library in the correct places for each platform.
#
# If INSTALL_DESTINATIONS is not empty, it will install to each of the destinations in the list.
# If INSTALL_DESTINATIONS is empty, this function does nothing.
#
# Arguments:
#	TARGET The name of the library target
function(InstallSharedLibrary)
	if(NOT INSTALL_DESTINATIONS)
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
	set(shared_lib_target "${INSTALL_SHARED_LIB_TARGET}")
	message(STATUS "Install shared library: ${shared_lib_target}")

	foreach (destination ${INSTALL_DESTINATIONS})
		if(UNIX AND NOT APPLE)
			set(destination ${LINUX_INSTALL_SHARED_DESTINATION})
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
				# In cmake 3.12:
				# list( JOIN VALID_TYPES ", " VALID_TYPES_STR )
				string( REPLACE ";" ", " VALID_TYPES_STR "${VALID_TYPES}" )
				
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
	if( "gl" IN_LIST VALID_TYPES )
		if( NOT INSTALL_PLUGINS_SHADER_DEST_PATH )
			message( FATAL_ERROR "InstallPlugins: SHADER_DEST_PATH not specified" )
		endif()
		
		message( STATUS " Shader Destination: ${INSTALL_PLUGINS_SHADER_DEST_PATH}/${INSTALL_PLUGINS_SHADER_DEST_FOLDER}" )
	endif()

	# Make CloudCompare/ccViewer depend on the plugins
	# so that when building CloudCompare/ccViewer the plugins also get built
	# instead of waiting for the `install` target to be ran for the plugins to get built
	add_dependencies(${PROJECT_NAME} ${CC_PLUGIN_TARGET_LIST})

	# Install the requested plugins in the DEST_FOLDER
	foreach( plugin_target ${CC_PLUGIN_TARGET_LIST} )
		get_target_property( plugin_type ${plugin_target} PLUGIN_TYPE )
		
		if( "${plugin_type}" IN_LIST INSTALL_PLUGINS_TYPES )
			message( STATUS " Install ${plugin_target} (${plugin_type})" )
			
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
endfunction()

# _InstallSharedTarget should only be called by one of the functions above.
# It was factored out to provide cmake < 3.13 a way to install shared libs.
#
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
	
	# Before CMake 3.13, install(TARGETS) would only accept targets created in the same directory scope
	# This makes it difficult to work with submodules.
	# This can be cleaned up when we move to a minimum CMake of 3.13 or higher
	# https://gitlab.kitware.com/cmake/cmake/-/merge_requests/2152
	if ( ${CMAKE_VERSION} VERSION_LESS "3.13.0" )
		# Basic hack: construct the name of the dynamic library ("target_shared_lib") and install using
		# install(FILES) instead of install(TARGETS)
		
		if ( APPLE OR UNIX )
			set( lib_prefix "lib" )
		endif()
		
		if ( CMAKE_BUILD_TYPE STREQUAL "Debug" )
			get_target_property( lib_postfix ${shared_target} DEBUG_POSTFIX)
		endif()
		
		get_target_property( target_bin_dir ${shared_target} BINARY_DIR )
		
		set( target_shared_lib "${target_bin_dir}/${lib_prefix}${shared_target}${lib_postfix}${CMAKE_SHARED_LIBRARY_SUFFIX}" )
				
		copy_files( "${target_shared_lib}" "${full_path}" 1 )

	else()	
		if( WIN32 )
			if( NOT CMAKE_CONFIGURATION_TYPES )
				install(
					TARGETS ${shared_target}
					RUNTIME DESTINATION ${full_path}
				)
			else()
				install(
					TARGETS ${shared_target}
					CONFIGURATIONS Debug
					RUNTIME DESTINATION ${INSTALL_SHARED_TARGET_DEST_PATH}_debug/${INSTALL_SHARED_TARGET_DEST_FOLDER}
				)
			
				install(
					TARGETS ${shared_target}
					CONFIGURATIONS Release
					RUNTIME DESTINATION ${full_path}
				)
			
				install(
					TARGETS ${shared_target}
					CONFIGURATIONS RelWithDebInfo
					RUNTIME DESTINATION ${INSTALL_SHARED_TARGET_DEST_PATH}_withDebInfo/${INSTALL_SHARED_TARGET_DEST_FOLDER}
				)
			endif()			
		else()
			install( TARGETS ${shared_target}
				LIBRARY DESTINATION ${full_path}
				COMPONENT Runtime
			)
		endif()
	endif()
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
	
	if( WIN32 )
		if( NOT CMAKE_CONFIGURATION_TYPES )
			install(
				FILES ${files}
				DESTINATION "${full_path}"
			)
		else()
			install(
				FILES ${files}
				CONFIGURATIONS Debug
				DESTINATION "${INSTALL_FILES_DEST_PATH}_debug/${INSTALL_FILES_DEST_FOLDER}"
			)
		
			install(
				FILES ${files}
				CONFIGURATIONS Release
				RUNTIME DESTINATION ${full_path}
			)
		
			install(
				FILES ${files}
				CONFIGURATIONS RelWithDebInfo
				RUNTIME DESTINATION "${INSTALL_FILES_DEST_PATH}_withDebInfo/${INSTALL_FILES_DEST_FOLDER}"
			)
		endif()			
	else()
		install(
			FILES ${files}
			DESTINATION "${full_path}"
		)
	endif()
endfunction()
