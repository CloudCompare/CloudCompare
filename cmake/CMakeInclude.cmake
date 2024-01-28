# ------------------------------------------------------------------------------
# helpers
# ------------------------------------------------------------------------------

# Copy files to the specified directory and for the active configurations
function( copy_files )	# 2 (or 3) arguments:
						# ARGV0 = files (if it's a list you have to provide the list alias quoted!)
						# ARGV1 = target (directory)
						# ARGV2 = 0 for release only install
						#         1 for both release and debug install (if available)
						#         2 for debug only install (if available)

	if ( WIN32 AND ${ARGC} LESS_EQUAL 2)
		message(WARNING "For Windows configurations, it's better to specify whether the file should be copied for both release only (0), release and debug (1) or debug only (2)")
	endif()

	if ( ${ARGC} LESS_EQUAL 2 OR NOT ${ARGV2} EQUAL 2)

		message(STATUS "Files: ${ARGV0} will be installed in ${ARGV1}" )
		
		if( WIN32 ) # Windows

			if( NOT CMAKE_CONFIGURATION_TYPES )
				install( FILES ${ARGV0} DESTINATION ${ARGV1} )
			else()
				install( FILES ${ARGV0} CONFIGURATIONS Release DESTINATION ${ARGV1} )
				install( FILES ${ARGV0} CONFIGURATIONS RelWithDebInfo DESTINATION ${ARGV1}_withDebInfo )
			endif()
		
		elseif() # macOS or Linux
			
			install( FILES ${ARGV0} DESTINATION ${ARGV1} )
			return()

		endif()
	endif()

	if ( ${ARGC} GREATER 2 )
		if ( ${ARGV2} EQUAL 1 OR ${ARGV2} EQUAL 2 )
			if(  NOT APPLE AND CMAKE_CONFIGURATION_TYPES )
				message(STATUS "Files: ${ARGV0} will be installed in ${ARGV1}_debug" )
				install( FILES ${ARGV0} CONFIGURATIONS Debug DESTINATION ${ARGV1}_debug )
			endif()
		endif()
	endif()

endfunction()

# Extended 'install' command depending on the build configuration and OS
# 4 arguments:
#   - ARGV0 = signature
#   - ARGV1 = target (warning: one project or one file at a time)
#   - ARGV2 = base install destination (_debug or _withDebInfo will be automatically appended if multi-conf is supported)
#   - ARGV3 = install destination suffix (optional)
function( install_ext )
	if( APPLE )
		install( ${ARGV0} ${ARGV1} DESTINATION ${ARGV2}${ARGV3} )
		return()
	endif()
	
	if( NOT CMAKE_CONFIGURATION_TYPES )
		install( ${ARGV0} ${ARGV1} DESTINATION ${ARGV2}${ARGV3} )
	else()
		install( ${ARGV0} ${ARGV1} CONFIGURATIONS Release DESTINATION ${ARGV2}${ARGV3} )
		install( ${ARGV0} ${ARGV1} CONFIGURATIONS RelWithDebInfo DESTINATION ${ARGV2}_withDebInfo${ARGV3} )
		install( ${ARGV0} ${ARGV1} CONFIGURATIONS Debug DESTINATION ${ARGV2}_debug${ARGV3} )
	endif()
endfunction()
