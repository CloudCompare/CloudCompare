# ALL 'contrib' supported libraries 

# PDAL support
include( contrib/PDALSupport.cmake )
# E57 support
include( contrib/E57Support.cmake )
# 3DXWARE (3dConnexion devices) support
include( contrib/3DXSupport.cmake )
# Gamepads support
include( contrib/GamepadSupport.cmake )
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

endfunction()
