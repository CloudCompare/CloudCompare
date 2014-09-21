cmake_minimum_required(VERSION 2.8)

# Eliminate a warning when building in Windows that relates
# to static linking of Qt executables to qtmain.lib.
# This policy was introduced in CMake version 2.8.11.
# CMake version 2.8.11.2 warns when the policy is not set
# and uses OLD behavior.
if(POLICY CMP0020)
  cmake_policy(SET CMP0020 OLD)
endif()

option( INSTALL_QPOISSON_RECON_PLUGIN "Check to install qPoissonRecon plugin" OFF )

if (INSTALL_QPOISSON_RECON_PLUGIN)

#CloudCompare 'PoissonRecon' plugin
project( QPOISSON_RECON_PLUGIN )

add_subdirectory (PoissonRecon6.11)

include( ../CMakePluginTpl.cmake )

target_link_libraries( ${PROJECT_NAME} QPOISSON_RECON_LIB )
include_directories( ${QPOISSON_RECON_LIB_SOURCE_DIR}/Src )
include_directories( ${QPOISSON_RECON_LIB_SOURCE_DIR}/Src_CC_wrap )

endif()
