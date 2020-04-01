cmake_minimum_required(VERSION 3.0)

option( PLUGIN_STANDARD_QRANSAC_SD "Check to install qRansac_SD (Shape Detection) plugin" OFF )

# CloudCompare 'Ransac Shape Detection' plugin
if (PLUGIN_STANDARD_QRANSAC_SD)
	project( QRANSAC_SD_PLUGIN )

	add_definitions( -DPOINTSWITHINDEX ) #Causes the sub clouds to be generated as partial clones of original cloud

	include( CMakePolicies NO_POLICY_SCOPE )


	add_subdirectory (RANSAC_SD_orig)

	include( ../../../CMakePluginTpl.cmake )

	target_link_libraries( ${PROJECT_NAME} QRANSAC_SD_PRIM_SHAPES_LIB QRANSAC_SD_MISC_LIB )

	include_directories( ${QRANSAC_SD_MISC_LIB_SOURCE_DIR} )
	include_directories( ${QRANSAC_SD_PRIM_SHAPES_LIB_SOURCE_DIR} )
	include_directories( ${QRANSAC_SD_PRIM_SHAPES_LIB_SOURCE_DIR}/GfxTL )

if (OPENMP_FOUND AND NOT WIN32) #DGM: OpenMP doesn't work with Visual at least (the process loops infinitely)
	target_compile_definitions( ${PROJECT_NAME} PRIVATE DOPARALLEL )
endif()

endif()
