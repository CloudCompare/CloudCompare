# ------------------------------------------------------------------------------
# helpers
# ------------------------------------------------------------------------------

# define CMake designation of shared libraries (DLL or so) whatever the OS is
if( WIN32 )
	set( SHARED_LIB_TYPE RUNTIME ) # CMake considers Dlls as RUNTIME on Windows!
else()
	set( SHARED_LIB_TYPE LIBRARY )
endif()

function( find_mocable_files __out_var_name )   # + input list
    set( local_list )
    foreach( one_file ${ARGN} )
        file( READ ${one_file} stream )
        if( stream MATCHES "Q_OBJECT" )
            list( APPEND local_list ${one_file} )
        endif()
    endforeach()
    set( ${__out_var_name} ${local_list} PARENT_SCOPE )
endfunction()

# Export Qt Dlls to specified destinations
function( install_Qt_Dlls ) # 2 arguments: ARGV0 = release destination / ARGV1 = debug destination
if( WIN32 )
if( NOT ${ARGC} EQUAL 2 )
	message( SEND_ERROR "function install_Qt_Dlls: invalid number of arguments! (need release and debug destinations)" )
else()
	set( QT_RELEASE_DLLS QtCore${QT_VERSION_MAJOR} QtGui${QT_VERSION_MAJOR} QtOpenGL${QT_VERSION_MAJOR} )
	set( QT_DEBUG_DLLS QtCored${QT_VERSION_MAJOR} QtGuid${QT_VERSION_MAJOR} QtOpenGLd${QT_VERSION_MAJOR} )
	#specific case for the MinGW version of Qts
	if( MINGW )
		list( APPEND QT_RELEASE_DLLS libgcc )
		list( APPEND QT_RELEASE_DLLS mingwm )
		list( APPEND QT_DEBUG_DLLS libgcc )
		list( APPEND QT_DEBUG_DLLS mingwm )
	endif()

	#release versions
	foreach( element ${QT_RELEASE_DLLS} )
		file( GLOB dll_files ${QT_BINARY_DIR}/${element}*.dll )
		foreach( qtDLL ${dll_files} )
			if( NOT CMAKE_CONFIGURATION_TYPES )
				install( FILES ${qtDLL} DESTINATION ${ARGV0} )
			else()
				install( FILES ${qtDLL} CONFIGURATIONS Release DESTINATION ${ARGV0} )
			endif()
		endforeach()
	endforeach()
	
	#debug versions (for mutli-config compiler only)
	if( CMAKE_CONFIGURATION_TYPES )
		foreach( element ${QT_DEBUG_DLLS} )
			file( GLOB dll_files ${QT_BINARY_DIR}/${element}*.dll )
			foreach( qtDLL ${dll_files} )
				install( FILES ${qtDLL} CONFIGURATIONS Debug DESTINATION ${ARGV1} )
			endforeach()
		endforeach()
	endif()
	
endif()
endif()
endfunction()

# Export Qt imageformats DLLs to specified destinations
function( install_Qt_ImageFormats ) # 2 arguments: ARGV0 = release destination / ARGV1 = debug destination
if( WIN32 )
if( NOT ${ARGC} EQUAL 2 )
	message( SEND_ERROR "function install_Qt_ImageFormats: invalid number of arguments! (need release and debug destinations)" )
else()
foreach( imagePlugin ${QT_IMAGEFORMATS_PLUGINS} )
	if( NOT CMAKE_CONFIGURATION_TYPES )
		install( FILES ${QT_PLUGINS_DIR}/imageformats/${imagePlugin}4.dll DESTINATION ${ARGV0}/imageformats )
	else()
		install( FILES ${QT_PLUGINS_DIR}/imageformats/${imagePlugin}4.dll CONFIGURATIONS Release DESTINATION ${ARGV0}/imageformats )
		install( FILES ${QT_PLUGINS_DIR}/imageformats/${imagePlugin}d4.dll CONFIGURATIONS Debug DESTINATION ${ARGV1}/imageformats )
	endif()
endforeach()
endif()
endif()
endfunction()

# Install shared libraries depending on the build configuration and OS
function( install_shared ) # 3 arguments: ARGV0 = target / ARGV1 = release install destination / ARGV2 = debug install destination (if available)
if( NOT CMAKE_CONFIGURATION_TYPES )
	install( TARGETS ${ARGV0} ${SHARED_LIB_TYPE} DESTINATION ${ARGV1} )
else()
	install( TARGETS ${ARGV0} ${SHARED_LIB_TYPE} CONFIGURATIONS Release DESTINATION ${ARGV1} )
	install( TARGETS ${ARGV0} ${SHARED_LIB_TYPE} CONFIGURATIONS Debug DESTINATION ${ARGV2} )
endif()
endfunction()

# Extended 'install' command depending on the build configuration and OS
# 4 arguments:
#   - ARGV0 = signature
#   - ARGV1 = target (warning: one project or one file at a time)
#   - ARGV2 = release install destination (or 0 to skip it)
#   - ARGV3 = debug install destination (if configuration exist / optional)
function( install_ext )
if( NOT CMAKE_CONFIGURATION_TYPES )
	install( ${ARGV0} ${ARGV1} DESTINATION ${ARGV2} )
else()
	if (NOT ${ARGV2} EQUAL 0)
		install( ${ARGV0} ${ARGV1} CONFIGURATIONS Release DESTINATION ${ARGV2} )
	endif()
	if( ARGV3 )
		install( ${ARGV0} ${ARGV1} CONFIGURATIONS Debug DESTINATION ${ARGV3} )
	endif()
endif()
endfunction()

# Default preprocessors
function( set_default_cc_preproc ) # 1 argument: ARGV0 = target
set( CC_DEFAULT_PREPROCESSORS NOMINMAX _CRT_SECURE_NO_WARNINGS ) #all configurations
set( CC_DEFAULT_PREPROCESSORS_RELEASE NDEBUG ) #release specific
set( CC_DEFAULT_PREPROCESSORS_DEBUG _DEBUG ) #debug specific
set_property( TARGET ${ARGV0} APPEND PROPERTY COMPILE_DEFINITIONS ${CC_DEFAULT_PREPROCESSORS} )
if( NOT CMAKE_CONFIGURATION_TYPES )
	set_property( TARGET ${ARGV0} APPEND PROPERTY COMPILE_DEFINITIONS ${CC_DEFAULT_PREPROCESSORS_RELEASE} )
else()
	set_property( TARGET ${ARGV0} APPEND PROPERTY COMPILE_DEFINITIONS_RELEASE ${CC_DEFAULT_PREPROCESSORS_RELEASE} )
	set_property( TARGET ${ARGV0} APPEND PROPERTY COMPILE_DEFINITIONS_DEBUG ${CC_DEFAULT_PREPROCESSORS_DEBUG} )
endif()
endfunction()
