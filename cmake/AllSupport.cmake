# ALL 'contrib' supported libraries

# Oculus support
include( OculusSupport )

function( target_link_contrib ) # 2 arguments: ARGV0 = project name / ARGV1 = shared lib export base folder (optional - '_debug' will be appended for debug configuration)

	#libraries with dynamic linkage can automatically 'install' their DLLs

	#OCULUS support
	#target_link_OCULUS_SDK( ${ARGV0} )

endfunction()
