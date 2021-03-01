# On Windows, we need to install the libraries that CGAL depends on.
if( WIN32 )
	# Because these are dependencies, we don't have access to the targets, so we
	# must work with files.
	# Use the variables set by the libraries themselves and call InstallFiles() on them.

	if( CCCORELIB_USE_TBB )
		InstallFiles( FILES ${TBB_LIBRARY_RELEASE} )
	endif()
	
	if( CCCORELIB_USE_CGAL )
	
		list(GET GMP_LIBRARIES 0 FIRST_GMP_LIB_FILE)
		get_filename_component(GMP_LIB_FOLDER ${FIRST_GMP_LIB_FILE} DIRECTORY)

		file( GLOB GMP_DLL_FILES ${GMP_LIB_FOLDER}/*.dll )
		foreach( dest ${INSTALL_DESTINATIONS} )
			copy_files( "${GMP_DLL_FILES}" ${dest} ) # Mind the quotes!
		endforeach()
	
		InstallFiles( FILES ${GMP_DLL_FILES} )
		#InstallFiles( FILES ${MPFR_LIBRARIES} )
	endif()
endif()
