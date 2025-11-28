# ------------------------------------------------------------------------------
# helpers
# ------------------------------------------------------------------------------

# Copy files to the specified directory and for the active configurations
function( copy_files )	# 2 (or 3) arguments:
						# ARGV0 = files (if it's a list you have to provide the list alias quoted!)
						# ARGV1 = target (directory)
	message(STATUS "Files: ${ARGV0} will be installed in ${ARGV1}" )
	install( FILES ${ARGV0} DESTINATION ${ARGV1} )
endfunction()

# Extended 'install' command depending on the build configuration and OS
# 4 arguments:
#   - ARGV0 = signature
#   - ARGV1 = target (warning: one project or one file at a time)
#   - ARGV2 = base install destination (_debug or _withDebInfo will be automatically appended if multi-conf is supported)
#   - ARGV3 = install destination suffix (optional)
function( install_ext )
	install( ${ARGV0} ${ARGV1} DESTINATION ${ARGV2}${ARGV3} )
endfunction()
