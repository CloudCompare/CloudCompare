# ------------------------------------------------------------------------------
# libE57Format+CMake support for CloudCompare
# ------------------------------------------------------------------------------

OPTION( OPTION_USE_LIBE57FORMAT "Build with libE57Format (ASTM E2807-11 E57 file format support)" OFF )

if( ${OPTION_USE_LIBE57FORMAT} )
	# libE57Format
	set( LIBE57FORMAT_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/contrib/libE57Format" CACHE PATH "libE57Format install directory (CMake INSTALL output)" )

	if( NOT LIBE57FORMAT_INSTALL_DIR )
		message( SEND_ERROR "No libE57Format install dir specified (LIBE57FORMAT_INSTALL_DIR)" )
	else()
		include_directories( ${LIBE57FORMAT_INSTALL_DIR}/include/E57Format )
	endif()
	
	# Find Xerces
	if (WIN32)
		set( Xerces_INCLUDE_DIR "" CACHE PATH "Xerces include directory" )
		set( Xerces_LIBRARY_RELEASE "" CACHE FILEPATH "Xerces (release) library file" )
 		if( CMAKE_CONFIGURATION_TYPES )
			set( Xerces_LIBRARY_DEBUG "" CACHE FILEPATH "Xerces (debug) library file" )
		endif()
	else ()
		include(FindXercesC)
		find_package(XercesC REQUIRED)

		set( Xerces_INCLUDE_DIR ${XercesC_INCLUDE_DIR} CACHE PATH "Xerces include directory" )
		set( Xerces_LIBRARY_RELEASE ${XercesC_LIBRARY} CACHE FILEPATH "Xerces (release) library file" )
		if( CMAKE_CONFIGURATION_TYPES )
			set( Xerces_LIBRARY_DEBUG ${XercesC_LIBRARY} CACHE FILEPATH "Xerces (debug) library file" )
		endif()
	endif()

	if ( NOT Xerces_INCLUDE_DIR )
		message( SEND_ERROR "No Xerces include dir specified (Xerces_INCLUDE_DIR)" )
	else()
		include_directories( ${Xerces_INCLUDE_DIR} )
	endif()

endif()

# link project with libE57Format libraries
function( target_link_LIBE57FORMAT ) # 1 argument: ARGV0 = project name
	if( ${OPTION_USE_LIBE57FORMAT} )
		if( LIBE57FORMAT_INSTALL_DIR )
			if (WIN32 AND NOT MINGW)
				set(LIBE57FORMAT_LIB_DEBUG "E57Format-d.lib")
				set(LIBE57FORMAT_LIB_RELEASE "E57Format.lib")
			else()
				set(LIBE57FORMAT_LIB_DEBUG "libE57Format-d.a")
				set(LIBE57FORMAT_LIB_RELEASE "libE57Format.a")
			endif()
			
			if ( CMAKE_CONFIGURATION_TYPES )
				target_link_libraries( ${ARGV0} debug ${LIBE57FORMAT_INSTALL_DIR}/lib/${LIBE57FORMAT_LIB_DEBUG} optimized ${LIBE57FORMAT_INSTALL_DIR}/lib/${LIBE57FORMAT_LIB_RELEASE} )
			else()
				target_link_libraries( ${ARGV0} ${LIBE57FORMAT_INSTALL_DIR}/lib/${LIBE57FORMAT_LIB_RELEASE} )
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
	
			set_property( TARGET ${ARGV0} APPEND PROPERTY COMPILE_DEFINITIONS CC_E57_SUPPORT XERCES_STATIC_LIBRARY )
		else()
			message( SEND_ERROR "No libE57Format install dir specified (LIBE57FORMAT_INSTALL_DIR)" )
		endif()
	endif()
endfunction()
