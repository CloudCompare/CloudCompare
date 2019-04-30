# ------------------------------------------------------------------------------
# GDAL support for CloudCompare
# ------------------------------------------------------------------------------

OPTION( OPTION_USE_VCGLIB "Build with vcglib support" OFF )
if( ${OPTION_USE_VCGLIB} )
	# try to find the vcg library
	set (VCGLIB_INCLUDE_DIR_HINTS "")
	find_package(vcglib REQUIRED)
	if (VCGLIB_FOUND)
		include_directories( ${VCGLIB_INCLUDE_DIRS} )
	  message(STATUS "vcglib found in: ${VCGLIB_INCLUDE_DIRS}")
	ENDIF (VCGLIB_FOUND)

	find_package(Eigen)
	if (EIGEN_FOUND)
		message(STATUS "Eigen found in:")
		message(STATUS ${EIGEN_INCLUDE_DIRS})
	ENDIF (EIGEN_FOUND)
endif()
