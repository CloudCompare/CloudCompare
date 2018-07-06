# ------------------------------------------------------------------------------
# helpers
# ------------------------------------------------------------------------------

# define CMake designation of shared libraries (DLL or so) whatever the OS is
if( WIN32 )
	set( SHARED_LIB_TYPE RUNTIME ) # CMake considers Dlls as RUNTIME on Windows!
else()
	set( SHARED_LIB_TYPE LIBRARY )
endif()

if( WIN32 )
	# Export Qt Dlls to specified destinations
	function( install_Qt_Dlls ) # 2 arguments: ARGV0 = base destination / ARGV1 = whether to add the QGamepad DLL or not
		if( ${ARGC} GREATER 0 )
			
			#All Qt Dlls (release mode)
			set(QT_RELEASE_DLLS)
			
			#standard DLLs (Qt 5)
			set( QT_RELEASE_DLLS_BASE_NAME Qt5Core Qt5Gui Qt5OpenGL Qt5Widgets Qt5Concurrent Qt5PrintSupport Qt5Svg )
			if( ${ARGC} GREATER 1 )
				if ( ${ARGV1} )
					list( APPEND QT_RELEASE_DLLS_BASE_NAME Qt5Gamepad )
				endif()
			endif()
			#ICU DLLs
			file( GLOB QT_RELEASE_DLLS ${QT_BINARY_DIR}/icu*.dll ) #first init the list with the ICU Dlls
	
			#specific case for the MinGW version of Qts
			if( MINGW )
				file ( GLOB QT_RELEASE_DLLS ${QT_BINARY_DIR}/libgcc*.dll )
				file ( GLOB QT_RELEASE_DLLS ${QT_BINARY_DIR}/libstdc++*.dll )
			endif()
	
			#generate full path of release Dlls
			foreach( element ${QT_RELEASE_DLLS_BASE_NAME} )
				#message(${element})
				list( APPEND QT_RELEASE_DLLS ${QT_BINARY_DIR}/${element}.dll)
			endforeach()
	
			foreach( qtDLL ${QT_RELEASE_DLLS} )
				if( NOT CMAKE_CONFIGURATION_TYPES )
					install( FILES ${qtDLL} DESTINATION ${ARGV0} )
				else()
					install( FILES ${qtDLL} CONFIGURATIONS Release DESTINATION ${ARGV0} )
				endif()
			endforeach()
			
			# for mutli-config compiler only
			if( CMAKE_CONFIGURATION_TYPES )
			
				#release with debug info version
				foreach( qtDLL ${QT_RELEASE_DLLS} )
					install( FILES ${qtDLL} CONFIGURATIONS RelWithDebInfo DESTINATION ${ARGV0}_withDebInfo )
				endforeach()
				
				#debug version
				set( QT_DEBUG_DLLS )
				
				#standard DLLs
				set( QT_DEBUG_DLLS_BASE_NAME Qt5Cored Qt5Guid Qt5OpenGLd Qt5Widgetsd Qt5Concurrentd Qt5PrintSupportd Qt5Svgd )
				if( ${ARGC} GREATER 1 )
					if ( ${ARGV1} )
						list( APPEND QT_DEBUG_DLLS_BASE_NAME Qt5Gamepadd )
					endif()
				endif()
				#ICU DLLs
				file( GLOB QT_DEBUG_DLLS ${QT_BINARY_DIR}/icu*.dll ) #first init the list with the ICU Dlls
	
				#specific case for the MinGW version of Qts
				if( MINGW )
					list( APPEND QT_DEBUG_DLLS_BASE_NAME libgcc )
					list( APPEND QT_DEBUG_DLLS_BASE_NAME mingwm )
				endif()
			
				#generate full path of release Dlls
				foreach( element ${QT_DEBUG_DLLS_BASE_NAME} )
					list( APPEND QT_DEBUG_DLLS ${QT_BINARY_DIR}/${element}.dll)
				endforeach()
	
				foreach( qtDLL ${QT_DEBUG_DLLS} )
					install( FILES ${qtDLL} CONFIGURATIONS Debug DESTINATION ${ARGV0}_debug )
				endforeach()
	
			endif()
		else()
			message( SEND_ERROR "function install_Qt_Dlls: invalid number of arguments! (need base destination)" )
		endif()
	endfunction()

	# Export Qt5 plugins to specified destinations
	function( install_Qt5_plugins )
		set( QT_PLUGINS_DIR ${QT5_ROOT_PATH}/plugins )
		set( platformPlugin qwindows )
		if( NOT CMAKE_CONFIGURATION_TYPES )
			install( FILES ${QT_PLUGINS_DIR}/platforms/${platformPlugin}.dll DESTINATION ${ARGV0}/platforms )
		else()
			install( FILES ${QT_PLUGINS_DIR}/platforms/${platformPlugin}.dll CONFIGURATIONS Release DESTINATION ${ARGV0}/platforms )
			install( FILES ${QT_PLUGINS_DIR}/platforms/${platformPlugin}.dll CONFIGURATIONS RelWithDebInfo DESTINATION ${ARGV0}_withDebInfo/platforms )
			install( FILES ${QT_PLUGINS_DIR}/platforms/${platformPlugin}d.dll CONFIGURATIONS Debug DESTINATION ${ARGV0}_debug/platforms )
		endif()
	endfunction()

	# Export Qt imageformats DLLs to specified destinations
	function( install_Qt_ImageFormats )
		set( QT_PLUGINS_DIR ${QT5_ROOT_PATH}/plugins )
		set( QT_IMAGEFORMATS_PLUGINS qgif qico qjpeg qsvg )
		foreach( imagePlugin ${QT_IMAGEFORMATS_PLUGINS} )
			if( NOT CMAKE_CONFIGURATION_TYPES )
				install( FILES ${QT_PLUGINS_DIR}/imageformats/${imagePlugin}${QT_VER_NUM}.dll DESTINATION ${ARGV0}/imageformats )
			else()
				install( FILES ${QT_PLUGINS_DIR}/imageformats/${imagePlugin}${QT_VER_NUM}.dll CONFIGURATIONS Release DESTINATION ${ARGV0}/imageformats )
				install( FILES ${QT_PLUGINS_DIR}/imageformats/${imagePlugin}${QT_VER_NUM}.dll CONFIGURATIONS RelWithDebInfo DESTINATION ${ARGV0}_withDebInfo/imageformats )
				install( FILES ${QT_PLUGINS_DIR}/imageformats/${imagePlugin}d${QT_VER_NUM}.dll CONFIGURATIONS Debug DESTINATION ${ARGV0}_debug/imageformats )
			endif()
		endforeach()
	endfunction()
endif()

# Install shared libraries depending on the build configuration and OS
function( install_shared )	# 3 arguments:
							# ARGV0 = target
							# ARGV1 = release install destination
							# ARGV2 = 1 for debug install (if available)
							# ARGV3 = suffix (optional)
	if( NOT CMAKE_CONFIGURATION_TYPES )
		install( TARGETS ${ARGV0} ${SHARED_LIB_TYPE} DESTINATION ${ARGV1}${ARGV3} )
	else()
		install( TARGETS ${ARGV0} ${SHARED_LIB_TYPE} CONFIGURATIONS Release DESTINATION ${ARGV1}${ARGV3} )
		install( TARGETS ${ARGV0} ${SHARED_LIB_TYPE} CONFIGURATIONS RelWithDebInfo DESTINATION ${ARGV1}_withDebInfo${ARGV3} )
		if (${ARGV2} EQUAL 1)
			install( TARGETS ${ARGV0} ${SHARED_LIB_TYPE} CONFIGURATIONS Debug DESTINATION ${ARGV1}_debug${ARGV3} )
		endif()
	endif()
endfunction()

# Copy files to the specified directory and for the active configurations
function( copy_files )	# 2 arguments:
						# ARGV0 = files (if it's a list you have to provide the list alias quoted!)
						# ARGV1 = target (directory)

	message(STATUS "Files " ${ARGV0} " will be installed to dest. " ${ARGV1})
	if( NOT CMAKE_CONFIGURATION_TYPES )
		install( FILES ${ARGV0} DESTINATION ${ARGV1} )
	else()
		install( FILES ${ARGV0} CONFIGURATIONS Release DESTINATION ${ARGV1} )
		install( FILES ${ARGV0} CONFIGURATIONS RelWithDebInfo DESTINATION ${ARGV1}_withDebInfo )
		install( FILES ${ARGV0} CONFIGURATIONS Debug DESTINATION ${ARGV1}_debug )
	endif()
endfunction()

# Extended 'install' command depending on the build configuration and OS
# 4 arguments:
#   - ARGV0 = signature
#   - ARGV1 = target (warning: one project or one file at a time)
#   - ARGV2 = base install destination (_debug or _withDebInfo will be automatically appended if multi-conf is supported)
#   - ARGV3 = install destination suffix (optional)
function( install_ext )
	if( NOT CMAKE_CONFIGURATION_TYPES )
		install( ${ARGV0} ${ARGV1} DESTINATION ${ARGV2}${ARGV3} )
	else()
		install( ${ARGV0} ${ARGV1} CONFIGURATIONS Release DESTINATION ${ARGV2}${ARGV3} )
		install( ${ARGV0} ${ARGV1} CONFIGURATIONS RelWithDebInfo DESTINATION ${ARGV2}_withDebInfo${ARGV3} )
		install( ${ARGV0} ${ARGV1} CONFIGURATIONS Debug DESTINATION ${ARGV2}_debug${ARGV3} )
	endif()
endfunction()
