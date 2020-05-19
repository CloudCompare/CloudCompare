# ALL 'contrib' supported libraries

# 3DXWARE (3dConnexion devices) support
include( 3DXSupport )
# Gamepads support
include( GamepadSupport )
# GDAL support
include( GDALSupport )
# Oculus support
include( OculusSupport )

function( target_link_contrib ) # 2 arguments: ARGV0 = project name / ARGV1 = shared lib export base folder (optional - '_debug' will be appended for debug configuration)

	#libraries with dynamic linkage can automatically 'install' their DLLs

	#GDAL support
	target_link_GDAL( ${ARGV0} ${ARGV1} )
	#OCULUS support
	#target_link_OCULUS_SDK( ${ARGV0} )

endfunction()
