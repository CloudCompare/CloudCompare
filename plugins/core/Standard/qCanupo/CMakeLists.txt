cmake_minimum_required(VERSION 3.0)

option( PLUGIN_STANDARD_QCANUPO "Check to install qCanupo plugin" OFF )

if (PLUGIN_STANDARD_QCANUPO)

	#CloudCompare 'Canupo' plugin
	project( QCANUPO_PLUGIN )
	
	include( CMakePolicies NO_POLICY_SCOPE )

	set( DLIB_ROOT "" CACHE PATH "DLib library root directory" )
	if ( NOT DLIB_ROOT )
		message( SEND_ERROR "No DLib library root specified (DLIB_ROOT)" )
	else()
		include_directories( ${DLIB_ROOT} )
	endif()
	
	include( ../../../CMakePluginTpl.cmake )
	
	option( COMPILE_QCANUPO_WITH_PRIVATE_PARTS "Check to compile qCanupo plugin with private methods (CNRS/UEB only)" OFF )
	if ( COMPILE_QCANUPO_WITH_PRIVATE_PARTS )
		target_compile_definitions( ${PROJECT_NAME} PRIVATE COMPILE_PRIVATE_CANUPO )
	endif()

endif()
