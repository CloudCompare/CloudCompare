# ------------------------------------------------------------------------------
# 3dConnexion+CMake support for CloudCompare
# ------------------------------------------------------------------------------

if (WIN32) #only supported on Windows for the moment!

OPTION( OPTION_SUPPORT_3DCONNEXION_DEVICES "Build with 3dConnexion libraries (3D mouses support)" OFF )
if( ${OPTION_SUPPORT_3DCONNEXION_DEVICES} )

	# 3DxWare (DGM: not necessary on Windows! Maybe for Linux or Mac versions?)
	# set( 3DXWARE_INCLUDE_DIR "" CACHE PATH "3DxWare include directory" )
	# set( 3DXWARE_LIB_DIR "" CACHE PATH "3DxWare libraries directory" )

	# if( NOT 3DXWARE_INCLUDE_DIR )
		# message( SEND_ERROR "3DXWARE include path is not specified (3DXWARE_INCLUDE_DIR)" )
	# else()
		# include_directories( ${3DXWARE_INCLUDE_DIR} )
	# endif()

endif()

# link project with 3DxWare libraries
function( target_link_3DXWARE ) # 1 argument: ARGV0 = project name

if( ${OPTION_SUPPORT_3DCONNEXION_DEVICES} )
	# (DGM: not necessary on Windows! Maybe for Linux or Mac versions?)
	# if( 3DXWARE_LIB_DIR )
		# target_link_libraries( ${ARGV0} debug ${3DXWARE_LIB_DIR}/siappD.lib optimized ${3DXWARE_LIB_DIR}/siapp.lib )
		set_property( TARGET ${ARGV0} APPEND PROPERTY COMPILE_DEFINITIONS CC_3DXWARE_SUPPORT )
	# else()
		# message( SEND_ERROR "3DXWARE library path is not specified (3DXWARE_LIB_DIR)" )
	# endif()
endif()

endfunction()

else()

function( target_link_3DXWARE ) # dummy function does nothing
endfunction()

endif()