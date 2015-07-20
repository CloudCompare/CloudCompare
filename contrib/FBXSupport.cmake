# ------------------------------------------------------------------------------
# Autodesk FBX support for CloudCompare
# ------------------------------------------------------------------------------

OPTION( OPTION_USE_FBX_SDK "Build with Autodesk FBX SDK support" OFF )
if(OPTION_USE_FBX_SDK)

	# FBX SDK
	set( FBX_SDK_INCLUDE_DIR "" CACHE PATH "FBX SDK include directory" )
	set( FBX_SDK_LIBRARY_FILE "" CACHE FILEPATH "FBX SDK static library file" )

	if( CMAKE_CONFIGURATION_TYPES )
		set( FBX_SDK_LIBRARY_FILE_DEBUG "" CACHE FILEPATH "FBX SDK static debug library file" )
	endif()

	if ( NOT FBX_SDK_INCLUDE_DIR )
		message( SEND_ERROR "No FBX SDK include dir specified (FBX_SDK_INCLUDE_DIR)" )
	else()
		include_directories( ${FBX_SDK_INCLUDE_DIR} )
	endif()
endif()

# Link project with FBX library
function( target_link_FBX_SDK ) # 2 arguments: ARGV0 = project name

if( ${OPTION_USE_FBX_SDK} )
	
	#release/general
	if( FBX_SDK_LIBRARY_FILE )

		if ( CMAKE_CONFIGURATION_TYPES )
			set_property( TARGET ${ARGV0} APPEND PROPERTY COMPILE_DEFINITIONS_RELEASE CC_FBX_SUPPORT )
			target_link_libraries( ${ARGV0} optimized ${FBX_SDK_LIBRARY_FILE} )
		else()
			set_property( TARGET ${ARGV0} APPEND PROPERTY COMPILE_DEFINITIONS CC_FBX_SUPPORT )
			target_link_libraries( ${ARGV0} ${FBX_SDK_LIBRARY_FILE} )
		endif()
		
	else()
	
		message( SEND_ERROR "FBX SDK library not found: can't link" )
	
	endif()

	#debug
	if ( CMAKE_CONFIGURATION_TYPES )
	
		if (FBX_SDK_LIBRARY_FILE_DEBUG)
			
			set_property( TARGET ${ARGV0} APPEND PROPERTY COMPILE_DEFINITIONS_DEBUG CC_FBX_SUPPORT )
			target_link_libraries( ${ARGV0} debug ${FBX_SDK_LIBRARY_FILE_DEBUG} )

		else()
		
			message( WARNING "No FBX SDK debug library file specified (FBX_SDK_LIBRARY_FILE_DEBUG)" )
		
		endif()
	
	endif()

endif()

endfunction()
