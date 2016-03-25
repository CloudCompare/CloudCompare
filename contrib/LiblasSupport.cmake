# ------------------------------------------------------------------------------
# LibLAS+CMake support for CloudCompare
# ------------------------------------------------------------------------------

OPTION( OPTION_USE_LIBLAS "Build with liblas support" OFF )
if( ${OPTION_USE_LIBLAS} )
	# Boost (using static, multithreaded libraries)
	if ( WIN32 )
		if ( MSVC )
			set(Boost_USE_STATIC_LIBS   ON)
			set(Boost_USE_MULTITHREADED ON)
		endif() 
	elseif( APPLE )
		set(Boost_USE_STATIC_LIBS   ON)
		set(Boost_USE_MULTITHREADED ON)
	endif()

	find_package(Boost 1.38 COMPONENTS program_options thread REQUIRED)

	if ( Boost_FOUND AND Boost_PROGRAM_OPTIONS_FOUND )
		include_directories( ${Boost_INCLUDE_DIRS} )
		link_directories( ${Boost_LIBRARY_DIRS} ) 
	endif()

	# make these available for the user to set.
	mark_as_advanced(CLEAR Boost_INCLUDE_DIR) 
	mark_as_advanced(CLEAR Boost_LIBRARY_DIRS) 

	# liblas
	set( LIBLAS_INCLUDE_DIR "" CACHE PATH "LibLAS include directory" )
	set( LIBLAS_RELEASE_LIBRARY_FILE "" CACHE FILEPATH "LibLAS library file (release mode)" )
	set( LIBLAS_DEBUG_LIBRARY_FILE "" CACHE FILEPATH "LibLAS library file (debug mode)" )
	if (WIN32)
		set( LIBLAS_SHARED_LIBRARY_FILE "" CACHE FILEPATH "LibLAS shared library file (dll release mode)" )
		set( LIBLAS_SHARED_DEBUG_LIBRARY_FILE "" CACHE FILEPATH "LibLAS shared library file (dll debug mode)" )
	endif()

	if ( NOT LIBLAS_INCLUDE_DIR )
		message( SEND_ERROR "No LibLAS include dir specified (LIBLAS_INCLUDE_DIR)" )
	else()
		include_directories( ${LIBLAS_INCLUDE_DIR} )
	endif()
endif()

# Link project with liblas library and export Dlls to specified destinations
function( target_link_liblas ) # 2 arguments: ARGV0 = project name / ARGV1 = shared lib export folder
	if( ${OPTION_USE_LIBLAS} )

		if( LIBLAS_RELEASE_LIBRARY_FILE )
			#Release mode only by default
			target_link_libraries( ${ARGV0} optimized ${LIBLAS_RELEASE_LIBRARY_FILE} )
			
			#export DLL
			if ( ARGV1 )
				if ( LIBLAS_SHARED_LIBRARY_FILE )
					string( REPLACE \\ / LIBLAS_SHARED_LIBRARY_FILE ${LIBLAS_SHARED_LIBRARY_FILE} )
					install( FILES ${LIBLAS_SHARED_LIBRARY_FILE} CONFIGURATIONS Release DESTINATION ${ARGV1} )
					if ( CMAKE_CONFIGURATION_TYPES )
						install( FILES ${LIBLAS_SHARED_LIBRARY_FILE} CONFIGURATIONS RelWithDebInfo DESTINATION ${ARGV1}_withDebInfo )
					endif()
				elseif( WIN32 )
					message( SEND_ERROR "No LibLAS DLL file specified (LIBLAS_SHARED_LIBRARY_FILE)" )
				endif()
			endif()
	
			#optional: debug mode
			if ( LIBLAS_DEBUG_LIBRARY_FILE )
				target_link_libraries( ${ARGV0} debug ${LIBLAS_DEBUG_LIBRARY_FILE} )
				#export DLL
				if ( ARGV1 )
					if ( LIBLAS_SHARED_DEBUG_LIBRARY_FILE )
						string( REPLACE \\ / LIBLAS_SHARED_DEBUG_LIBRARY_FILE ${LIBLAS_SHARED_DEBUG_LIBRARY_FILE} )
						install( FILES ${LIBLAS_SHARED_DEBUG_LIBRARY_FILE} CONFIGURATIONS Debug DESTINATION ${ARGV1}_debug )
					endif()
				endif()
			endif()
			
			if ( CMAKE_CONFIGURATION_TYPES )

				#Anytime we use COMPILE_DEFINITIONS_XXX we must define this policy!
				#(and setting it outside of the function/file doesn't seem to work...)
				cmake_policy(SET CMP0043 OLD)

				set_property( TARGET ${ARGV0} APPEND PROPERTY COMPILE_DEFINITIONS_RELEASE CC_LAS_SUPPORT )
				set_property( TARGET ${ARGV0} APPEND PROPERTY COMPILE_DEFINITIONS_RELWITHDEBINFO CC_LAS_SUPPORT )
				
				if ( LIBLAS_DEBUG_LIBRARY_FILE )
					set_property( TARGET ${ARGV0} APPEND PROPERTY COMPILE_DEFINITIONS_DEBUG CC_LAS_SUPPORT )
				endif()
			else()
				set_property( TARGET ${ARGV0} APPEND PROPERTY COMPILE_DEFINITIONS CC_LAS_SUPPORT )
			endif()
		else() #if( NOT LIBLAS_RELEASE_LIBRARY_FILE )
			message( SEND_ERROR "No LibLAS release library file specified (LIBLAS_RELEASE_LIBRARY_FILE)" )
		endif()
	endif()
endfunction()
