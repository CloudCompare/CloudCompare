cmake_minimum_required(VERSION 2.8)

option( INSTALL_QPOISSON_RECON_PLUGIN "Check to install qPoissonRecon plugin" OFF )

if (INSTALL_QPOISSON_RECON_PLUGIN)

#CloudCompare 'PoissonRecon' plugin
project( QPOISSON_RECON_PLUGIN )

option( POISSON_RECON_WITH_OPEN_MP "Check to compile PoissonRecon plugin with OpenMP support" OFF )

add_subdirectory (PoissonReconLib)

include( ../CMakePluginTpl.cmake )

target_link_libraries( ${PROJECT_NAME} QPOISSON_RECON_LIB )
include_directories( ${QPOISSON_RECON_LIB_SOURCE_DIR}/Src )
include_directories( ${QPOISSON_RECON_LIB_SOURCE_DIR}/Src_CC_wrap )

if ( POISSON_RECON_WITH_OPEN_MP )
	find_package(OpenMP)
	if (OPENMP_FOUND)
		set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
		set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
		set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS WITH_OPENMP )
	endif()
endif ()

endif()
