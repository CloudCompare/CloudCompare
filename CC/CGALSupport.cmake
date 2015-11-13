# ------------------------------------------------------------------------------
# CGAL+CMake support for CloudCompare
# ------------------------------------------------------------------------------

FIND_PACKAGE(CGAL QUIET COMPONENTS Core ) # implies findGMP

if (CGAL_FOUND)
	if(${CGAL_MAJOR_VERSION}.${CGAL_MINOR_VERSION} LESS 4.3)
		message(SEND_ERROR "CC Lib requires at least CGAL 4.3")
	endif()
	
	include( ${CGAL_USE_FILE} )
	include_directories(${CGAL_INCLUDE_DIR})
	
	#take care of GMP and MPFR DLLs on Windows!
	if( WIN32 )
		#message(${GMP_LIBRARIES})
		list(GET GMP_LIBRARIES 0 FIRST_GMP_LIB_FILE)
		get_filename_component(GMP_LIB_FOLDER ${FIRST_GMP_LIB_FILE} DIRECTORY)
		#message(${GMP_LIB_FOLDER})

		file( GLOB GMP_DLL_FILES ${GMP_LIB_FOLDER}/*.dll )
		foreach( dest ${INSTALL_DESTINATIONS} )
			copy_files( "${GMP_DLL_FILES}" ${dest} ) #mind the quotes!
		endforeach()
	endif()

else()

	message(SEND_ERROR "Could not find CGAL")

endif()
