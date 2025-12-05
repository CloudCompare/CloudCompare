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
