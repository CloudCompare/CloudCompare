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
	if( NOT Boost_FOUND )
		set( BOOST_ROOT CACHE PATH "Location of the boost root directory" )
		message( FATAL_ERROR "Unable to find boost library. Please set the BOOST_ROOT to point to the boost distribution files." )
	endif()
	
	# Find Xerces
	# we use libE57 own Xerces find module
	set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/contrib/cmake/Modules/" )

	set( Xerces_USE_STATIC_LIBS ON )
	find_package( Xerces QUIET )
	if ( NOT Xerces_FOUND )
		set( XERCES_ROOT CACHE PATH "Location of the Xerces library" )
		message( FATAL_ERROR "Unable to find Xerces library. Please make XERCES_ROOT point to the root of Xerces directory." )
	else()
		include_directories( ${Xerces_INCLUDE_DIR} ${Boost_INCLUDE_DIR} )
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
		target_link_libraries( ${ARGV0} debug ${LIBE57_INSTALL_DIR}/lib/E57RefImpl-d.lib optimized ${LIBE57_INSTALL_DIR}/lib/E57RefImpl.lib )
		#Xerces
		target_link_libraries( ${ARGV0} debug ${Xerces_LIBRARY_DEBUG} optimized ${Xerces_LIBRARY_RELEASE} ${Boost_LIBRARIES} )
		#Boost
		#link_directories( ${Boost_LIBRARY_DIRS} )

		set_property( TARGET ${ARGV0} APPEND PROPERTY COMPILE_DEFINITIONS CC_E57_SUPPORT XERCES_STATIC_LIBRARY )
	else()
		message( SEND_ERROR "No LibE57 install dir specified (LIBE57_INSTALL_DIR)" )
	endif()
endif()

endfunction()

