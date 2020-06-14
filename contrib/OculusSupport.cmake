# ------------------------------------------------------------------------------
# Oculus LibOVR support for CloudCompare
# ------------------------------------------------------------------------------

OPTION( OPTION_USE_OCULUS_SDK "Build with Oculus SDK (LibOVR) support" OFF )
if(OPTION_USE_OCULUS_SDK)

	# Oculus SDK (LibOVR)
	set( OCULUS_SDK_INCLUDE_DIR "" CACHE PATH "Oculus SDK (LibOVR) include directory" )
	set( OCULUS_SDK_LIBRARY_FILE "" CACHE FILEPATH "Oculus SDK (LibOVR) static library file" )
	
	if ( NOT OPTION_GL_QUAD_BUFFER_SUPPORT )
		message( SEND_ERROR "OPTION_GL_QUAD_BUFFER_SUPPORT must be enabled to support Oculus" )
	endif()	

	if ( NOT OCULUS_SDK_INCLUDE_DIR )
		message( SEND_ERROR "No Oculus SDK include dir specified (OCULUS_SDK_INCLUDE_DIR)" )
	else()
		include_directories( ${OCULUS_SDK_INCLUDE_DIR} )
	endif()
endif()

# Link project with LibOVR library
function( target_link_OCULUS_SDK ) # 1 argument: ARGV0 = project name
	if( ${OPTION_USE_OCULUS_SDK} )
		#release/general
		if( OCULUS_SDK_LIBRARY_FILE )
			target_compile_definitions( ${ARGV0} PUBLIC CC_OCULUS_SUPPORT )
			target_link_libraries( ${ARGV0} ${OCULUS_SDK_LIBRARY_FILE} )
		else()
			message( SEND_ERROR "FBX SDK library not found: can't link" )
		endif()
	endif()
endfunction()
