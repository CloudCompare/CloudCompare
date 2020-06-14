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
					target_link_libraries( ${ARGV0} debug ${3DXWARE_LIB_DIR}/siapp.lib ${3DXWARE_LIB_DIR}/spwmathD.lib )
				endif()
				
				target_compile_definitions( ${ARGV0} PUBLIC CC_3DXWARE_SUPPORT )
			else()
				message( SEND_ERROR "3DXWARE libraries folder is not specified (3DXWARE_LIB_DIR)" )
			endif()

		endif()
	endif()

endfunction()
