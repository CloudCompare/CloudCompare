# ------------------------------------------------------------------------------
# 3dConnexion+CMake support for CloudCompare
# ------------------------------------------------------------------------------

OPTION( OPTION_SUPPORT_3DCONNEXION_DEVICES "Build with 3dConnexion libraries (3D mouses support)" OFF )
if( ${OPTION_SUPPORT_3DCONNEXION_DEVICES} )

	# 3DxWare
	set( 3DXWARE_INCLUDE_DIR "" CACHE PATH "3DxWare include directory" )
	set( 3DXWARE_LIB_DIR "" CACHE PATH "3DxWare libraries directory" )

	if( NOT 3DXWARE_INCLUDE_DIR )
		message( SEND_ERROR "3DXWARE include path is not specified (3DXWARE_INCLUDE_DIR)" )
	else()
		include_directories( ${3DXWARE_INCLUDE_DIR} )
	endif()

endif()

# link project with 3DxWare libraries
function( target_link_3DXWARE ) # 1 argument: ARGV0 = project name

	if (WIN32) #only supported on Windows for the moment!
		if( ${OPTION_SUPPORT_3DCONNEXION_DEVICES} )
			if( 3DXWARE_LIB_DIR )
			
				target_link_libraries( ${ARGV0} optimized ${3DXWARE_LIB_DIR}/siapp.lib ${3DXWARE_LIB_DIR}/spwmath.lib)
			
				if ( CMAKE_CONFIGURATION_TYPES )
				
					target_link_libraries( ${ARGV0} debug ${3DXWARE_LIB_DIR}/siappD.lib ${3DXWARE_LIB_DIR}/spwmathD.lib )

					#Anytime we use COMPILE_DEFINITIONS_XXX we must define this policy!
					#(and setting it outside of the function/file doesn't seem to work...)
					cmake_policy(SET CMP0043 OLD)

					set_property( TARGET ${ARGV0} APPEND PROPERTY COMPILE_DEFINITIONS_RELEASE CC_3DXWARE_SUPPORT )
					set_property( TARGET ${ARGV0} APPEND PROPERTY COMPILE_DEFINITIONS_RELWITHDEBINFO CC_3DXWARE_SUPPORT )
					set_property( TARGET ${ARGV0} APPEND PROPERTY COMPILE_DEFINITIONS_DEBUG CC_3DXWARE_SUPPORT )
				else()
					set_property( TARGET ${ARGV0} APPEND PROPERTY COMPILE_DEFINITIONS CC_3DXWARE_SUPPORT )
				endif()
			else()
				message( SEND_ERROR "3DXWARE libraries folder is not specified (3DXWARE_LIB_DIR)" )
			endif()

		endif()
	endif()

endfunction()
