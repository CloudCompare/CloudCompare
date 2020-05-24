# On Windows, we need to install the libraries that CGAL depends on.
if( WIN32 )
	# Because these are dependencies, we don't have access to the targets, so we
	# must work with files.
	# Use the variables set by the libraries themselves and call InstallFiles() on them.

	if( CCCORELIB_USE_TBB )
		InstallFiles( FILES ${TBB_LIBRARY_RELEASE} )
	endif()
	
	if( CCCORELIB_USE_CGAL )
		InstallFiles( FILES ${GMP_LIBRARIES} )
		InstallFiles( FILES ${MPFR_LIBRARIES} )
	endif()
endif()
