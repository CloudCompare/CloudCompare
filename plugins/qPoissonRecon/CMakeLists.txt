cmake_minimum_required(VERSION 2.8)

option( INSTALL_QPOISSON_RECON_PLUGIN "Check to install qPoissonRecon plugin" OFF )

if (INSTALL_QPOISSON_RECON_PLUGIN)

#CloudCompare 'PoissonRecon' plugin
project( QPOISSON_RECON_PLUGIN )

add_subdirectory (PoissonReconLib)

include( ../CMakePluginTpl.cmake )

target_link_libraries( ${PROJECT_NAME} QPOISSON_RECON_LIB )
include_directories( ${QPOISSON_RECON_LIB_SOURCE_DIR}/Src )
include_directories( ${QPOISSON_RECON_LIB_SOURCE_DIR}/Src_CC_wrap )

endif()
