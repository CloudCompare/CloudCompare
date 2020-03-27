cmake_minimum_required(VERSION 3.0)

option( PLUGIN_STANDARD_QPOISSON_RECON "Check to install qPoissonRecon plugin" OFF )

# CloudCompare 'PoissonRecon' plugin
if (PLUGIN_STANDARD_QPOISSON_RECON)
	project( QPOISSON_RECON_PLUGIN )
		
	include( CMakePolicies NO_POLICY_SCOPE )

	add_subdirectory (PoissonReconLib)

	include( ../../../CMakePluginTpl.cmake )
	
	target_link_libraries( ${PROJECT_NAME} QPOISSON_RECON_LIB )
	include_directories( ${QPOISSON_RECON_LIB_SOURCE_DIR}/Src_CC_wrap )

endif()
