# ALL 'contrib' supported libraries 

# liblas support
#include( contrib/LiblasSupport.cmake )
# PDAL support
include( contrib/PDALSupport.cmake )
# E57 support
include( contrib/E57Support.cmake )
# 3DXWARE (3dConnexion devices) support
include( contrib/3DXSupport.cmake )
# Gamepads support
include( contrib/GamepadSupport.cmake )
# PDMS support
OPTION( OPTION_SUPPORT_MAC_PDMS_FORMAT "Build with .mac PDMS format" OFF )
# DXF support
include( contrib/DxfLibSupport.cmake )
# GDAL support
include( contrib/GDALSupport.cmake )
# FBX support
include( contrib/FBXSupport.cmake )
# SHP support
include( contrib/ShapeLibSupport.cmake )
# Oculus support
include( contrib/OculusSupport.cmake )

function( target_link_contrib ) # 2 arguments: ARGV0 = project name / ARGV1 = shared lib export base folder (optional - '_debug' will be appended for debug configuration)

	#libraries with dynamic linkage can automatically 'install' their DLLs

	#liblas support
	#target_link_liblas( ${ARGV0} ${ARGV1} )
	#PDAL support
	target_link_PDAL( ${ARGV0} ${ARGV1} )
	#GDAL support
	target_link_GDAL( ${ARGV0} ${ARGV1} )
	#E57 support
	target_link_LIBE57FORMAT( ${ARGV0} )
	#DXF support
	target_link_DXFLIB( ${ARGV0} )
	#FBX support
	target_link_FBX_SDK( ${ARGV0} )
	#SHP support
	target_link_SHAPE_LIB( ${ARGV0} )
	#OCULUS support
	#target_link_OCULUS_SDK( ${ARGV0} )
	
	# PDMS support (see qCC_io)
	if( ${OPTION_SUPPORT_MAC_PDMS_FORMAT} )
		set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS CC_PDMS_SUPPORT )
	endif()

endfunction()
