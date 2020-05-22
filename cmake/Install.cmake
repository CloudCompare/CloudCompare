# InstallSharedLibrary should be called once for each shared library in the libs directory.
# This function installs the shared library in the correct places for each platform.
# For macOS and Windows, it puts them with each application (CloudCompare and ccViewer).
# For Linux, it puts it in the proper place for shared libraries so both applcations can
# access them.
#
# Arguments:
#	TARGET The name of the library target
function( InstallSharedLibrary )
	cmake_parse_arguments(
			INSTALL_SHARED_LIB
			""
			"TARGET"
			""
			${ARGN}
	)

	# For readability
	set( shared_lib_target "${INSTALL_SHARED_LIB_TARGET}" )

	message( STATUS "Install shared library: ${shared_lib_target}")

	if( WIN32 )
		foreach( destination ${INSTALL_DESTINATIONS} )
			install_shared( ${shared_lib_target} ${destination} 1 )
		endforeach()
	elseif( APPLE )
		foreach( destination ${INSTALL_DESTINATIONS} )
			install( TARGETS ${shared_lib_target}
				LIBRARY DESTINATION ${destination}
				COMPONENT Runtime
			)
		endforeach()
	else()
		install( TARGETS ${shared_lib_target}
			LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/cloudcompare
			COMPONENT Runtime
		)
	endif()
endfunction()

# InstallPlugins should be called once for each application.
# This function installs the plugin types that are requested in TYPES to the path specified by DEST_FOLDER.
# If it is a gl plugin with shaders, install the shaders to SHADER_DEST_FOLDER.
#
# Arguments:
#	DEST_FOLDER Where to install the plugins.
#	SHADER_DEST_FOLDER Where to install the shaders for the plugins.
#	TYPES Semicolon-separated list of plugin types to install (valid: gl, io, standard). If not specified, install all.
function( InstallPlugins )
	cmake_parse_arguments(
			INSTALL_PLUGINS
			""
			"DEST_FOLDER;SHADER_DEST_FOLDER"
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
	
	# Check our destination folder is valid
	if( NOT INSTALL_PLUGINS_DEST_FOLDER )
		message( FATAL_ERROR "InstallPlugins: DEST_FOLDER not specified" )
	endif()
	
	message( STATUS " Destination: ${INSTALL_PLUGINS_DEST_FOLDER}" )
	
	# If we have gl plugins, check that our shader destination folder is valid
	if( "gl" IN_LIST VALID_TYPES )
		if( NOT INSTALL_PLUGINS_SHADER_DEST_FOLDER )
			message( FATAL_ERROR "InstallPlugins: SHADER_DEST_FOLDER not specified" )
		endif()
		
		message( STATUS " Shader Destination: ${INSTALL_PLUGINS_SHADER_DEST_FOLDER}" )
	endif()
	
	# Install the requested plugins in the DEST_FOLDER
	foreach( plugin_target ${CC_PLUGIN_TARGET_LIST} )
		get_target_property( plugin_type ${plugin_target} PLUGIN_TYPE )
		
		if( "${plugin_type}" IN_LIST INSTALL_PLUGINS_TYPES )
			message( STATUS " Install ${plugin_target} (${plugin_type})" )
			
			_InstallPluginTarget(
				TARGET ${plugin_target}
				DEST_FOLDER ${INSTALL_PLUGINS_DEST_FOLDER}
			)		
			
			if( "${plugin_type}" STREQUAL "gl" )
				get_target_property( SHADER_FOLDER_NAME ${plugin_target} SHADER_FOLDER_NAME )
				get_target_property( SHADER_FOLDER_PATH ${plugin_target} SHADER_FOLDER_PATH )
				
				if( EXISTS "${SHADER_FOLDER_PATH}" )
					message( STATUS "  + shader: ${SHADER_FOLDER_NAME} (${SHADER_FOLDER_PATH})" )
					
					get_target_property( shader_files ${plugin_target} SOURCES )
					list( FILTER shader_files INCLUDE REGEX ".*\.vert|frag" )					
					
					foreach( filename ${shader_files} )
						install(
							FILES ${filename}
							DESTINATION "${INSTALL_PLUGINS_SHADER_DEST_FOLDER}/${SHADER_FOLDER_NAME}"
						)
					endforeach()
				endif()
			endif()
		endif()
	endforeach()	
endfunction()

# _InstallPluginTarget should only be called by InstallPlugins above.
# It was factored out to provide cmake < 3.13 a way to install plugins.
#
# Arguments:
#	DEST_FOLDER Where to install the plugins.
#	TARGET The name of the plugin target
function( _InstallPluginTarget )
	cmake_parse_arguments(
			INSTALL_PLUGIN_TARGET
			""
			"DEST_FOLDER;TARGET"
			""
			${ARGN}
	)
	
	# For readability
	set( plugin_target "${INSTALL_PLUGIN_TARGET_TARGET}" )
	
	# Before CMake 3.13, install(TARGETS) would only accept targets created in the same directory scope
	# This makes it difficult to work with submodules.
	# This can be cleaned up when we move to a minimum CMake of 3.13 or higher
	# https://gitlab.kitware.com/cmake/cmake/-/merge_requests/2152
	if ( ${CMAKE_VERSION} VERSION_LESS "3.13.0" )
		# Basic hack: construct the name of the plugin dynamic library ("target_plugin") and install using
		# install(FILES) instead of install(TARGETS)
		
		if ( APPLE OR UNIX )
			set( lib_prefix "lib" )
		endif()
		
		if ( CMAKE_BUILD_TYPE STREQUAL "Debug" )
			get_target_property( lib_postfix ${plugin_target} DEBUG_POSTFIX)
		endif()
		
		get_target_property( target_bin_dir ${plugin_target} BINARY_DIR )
		
		set( target_plugin "${target_bin_dir}/${lib_prefix}${plugin_target}${lib_postfix}${CMAKE_SHARED_LIBRARY_SUFFIX}" )
				
		if ( WIN32 )
			copy_files( "${target_plugin}" "${INSTALL_PLUGIN_TARGET_DEST_FOLDER}" 1 )
		else()
			copy_files( "${target_plugin}" "${INSTALL_PLUGIN_TARGET_DEST_FOLDER}" )
		endif()
	else()	
		if( WIN32 )
			install_shared( ${plugin_target} ${INSTALL_PLUGIN_TARGET_DEST_FOLDER} 1 )
		else()
			install( TARGETS ${plugin_target}
				LIBRARY DESTINATION ${INSTALL_PLUGIN_TARGET_DEST_FOLDER}
				COMPONENT Runtime
			)
		endif()
	endif()
endfunction()
