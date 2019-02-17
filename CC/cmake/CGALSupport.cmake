# ------------------------------------------------------------------------------
# CGAL+CMake support for CloudCompare
# ------------------------------------------------------------------------------

FIND_PACKAGE( CGAL QUIET COMPONENTS Core ) # implies findGMP

if (CGAL_FOUND)
	if(${CGAL_MAJOR_VERSION} LESS 4)
		message(SEND_ERROR "CC Lib requires at least CGAL 4.3")
	endif()
	if(${CGAL_MAJOR_VERSION} EQUAL 4 AND CGAL_MINOR_VERSION LESS 3)
		message(SEND_ERROR "CC Lib requires at least CGAL 4.3")
	endif()

	# We need to get rid of CGAL CXX flags
	set(CGAL_DONT_OVERRIDE_CMAKE_FLAGS ON CACHE INTERNAL "override CGAL flags" FORCE)
	set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE CACHE INTERNAL "turn off warning")
	
	include( ${CGAL_USE_FILE} )
	include_directories(${CGAL_INCLUDE_DIR})

	# Take care of GMP and MPFR DLLs on Windows!
	if( WIN32 )
		# message(${GMP_LIBRARIES})
		list(GET GMP_LIBRARIES 0 FIRST_GMP_LIB_FILE)
		get_filename_component(GMP_LIB_FOLDER ${FIRST_GMP_LIB_FILE} DIRECTORY)
		# message(${GMP_LIB_FOLDER})

		file( GLOB GMP_DLL_FILES ${GMP_LIB_FOLDER}/*.dll )
		foreach( dest ${INSTALL_DESTINATIONS} )
			copy_files( "${GMP_DLL_FILES}" ${dest} ) # Mind the quotes!
		endforeach()
	endif()

else()

	message(SEND_ERROR "Could not find CGAL")

endif()
