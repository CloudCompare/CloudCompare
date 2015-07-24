# ------------------------------------------------------------------------------
# libE57+CMake support for CloudCompare
# ------------------------------------------------------------------------------

OPTION( OPTION_USE_LIBE57 "Build with libE57 (ASTM E2807-11 E57 file format support)" OFF )
if( ${OPTION_USE_LIBE57} )

	# DGM: does not work on my machine!
	#find_package( E57RefImpl )

	# LibE57
	set( LIBE57_INSTALL_DIR "" CACHE PATH "libE57 install directory (CMake INSTALL output)" )

	if( NOT LIBE57_INSTALL_DIR )
		message( SEND_ERROR "No LibE57 install dir specified (LIBE57_INSTALL_DIR)" )
	else()
		include_directories( ${LIBE57_INSTALL_DIR}/include )
		include_directories( ${LIBE57_INSTALL_DIR}/include/e57 )
	endif()

	# Find Boost
	find_package( Boost COMPONENTS system QUIET ) #DGM: not sure why, but "system" lib doesn't show up otherwise...
	if( Boost_FOUND )
		include_directories( ${Boost_INCLUDE_DIR} )
	else()
		set( BOOST_ROOT CACHE PATH "Location of the boost root directory" )
		message( FATAL_ERROR "Unable to find boost library. Please set BOOST_ROOT to point to the boost distribution files." )
	endif()
	
	# Find Xerces
	set( Xerces_INCLUDE_DIR "" CACHE PATH "Xerces include directory" )
	set( Xerces_LIBRARY_RELEASE "" CACHE FILEPATH "Xerces (release) library file" )
	if( CMAKE_CONFIGURATION_TYPES )
		set( Xerces_LIBRARY_DEBUG "" CACHE FILEPATH "Xerces (debug) library file" )
	endif()

	if ( NOT Xerces_INCLUDE_DIR )
		message( SEND_ERROR "No Xerces include dir specified (Xerces_INCLUDE_DIR)" )
	else()
		include_directories( ${Xerces_INCLUDE_DIR} )
	endif()

endif()

# link project with LIBE57 libraries
function( target_link_LIBE57 ) # 1 argument: ARGV0 = project name

if( ${OPTION_USE_LIBE57} )
	# with 'find_package'
	# if ( E57RefImpl_SCANLIB_LIBRARIES )
	# 	target_link_libraries( ${ARGV0} ${E57RefImpl_SCANLIB_LIBRARIES} )
	# endif()

	# manual version
	if( LIBE57_INSTALL_DIR )

	#libE57
		if (WIN32)
			set(LIBE57_LIB_DEBUG "E57RefImpl-d.lib")
			set(LIBE57_LIB_RELEASE "E57RefImpl.lib")
		else()
			set(LIBE57_LIB_DEBUG "libE57RefImpl-d.a")
			set(LIBE57_LIB_RELEASE "libE57RefImpl.a")
		endif()
		
		if ( CMAKE_CONFIGURATION_TYPES )
			target_link_libraries( ${ARGV0} debug ${LIBE57_INSTALL_DIR}/lib/${LIBE57_LIB_DEBUG} optimized ${LIBE57_INSTALL_DIR}/lib/${LIBE57_LIB_RELEASE} )
		else()
			target_link_libraries( ${ARGV0} ${LIBE57_INSTALL_DIR}/lib/${LIBE57_LIB_RELEASE} )
		endif()
		
		#Xerces
		if ( CMAKE_CONFIGURATION_TYPES )
			if (Xerces_LIBRARY_DEBUG AND Xerces_LIBRARY_RELEASE)
				target_link_libraries( ${ARGV0} debug ${Xerces_LIBRARY_DEBUG} optimized ${Xerces_LIBRARY_RELEASE} )
			else()
				message( FATAL_ERROR "Unable to find Xerces library. Please set Xerces_LIBRARY_DEBUG and Xerces_LIBRARY_RELEASE to point to the release and debug library files." )
			endif()
		else()
			if (Xerces_LIBRARY_RELEASE)
				target_link_libraries( ${ARGV0} ${Xerces_LIBRARY_RELEASE} )
			else()
				message( FATAL_ERROR "Unable to find Xerces library. Please set Xerces_LIBRARY_RELEASE to point to the (release) library file." )
			endif()
		endif()
		
		#Boost
		target_link_libraries( ${ARGV0} ${Boost_LIBRARIES} )

		set_property( TARGET ${ARGV0} APPEND PROPERTY COMPILE_DEFINITIONS CC_E57_SUPPORT XERCES_STATIC_LIBRARY )
	else()
		message( SEND_ERROR "No LibE57 install dir specified (LIBE57_INSTALL_DIR)" )
	endif()
endif()

endfunction()

