cmake_minimum_required(VERSION 2.8)

# Eliminate a warning when building in Windows that relates
# to static linking of Qt executables to qtmain.lib.
# This policy was introduced in CMake version 2.8.11.
# CMake version 2.8.11.2 warns when the policy is not set
# and uses OLD behavior.
if(POLICY CMP0020)
  cmake_policy(SET CMP0020 OLD)
endif()

option( INSTALL_QRANSAC_SD_PLUGIN "Check to install qRansac_SD (Shape Detection) plugin" OFF )

if (INSTALL_QRANSAC_SD_PLUGIN)

#CloudCompare 'Ransac Shape Detection' plugin
project( QRANSAC_SD_PLUGIN )

add_subdirectory (RANSAC_SD_orig)

include( ../CMakePluginTpl.cmake )

target_link_libraries( ${PROJECT_NAME} QRANSAC_SD_PRIM_SHAPES_LIB QRANSAC_SD_MISC_LIB )

include_directories( ${QRANSAC_SD_MISC_LIB_SOURCE_DIR} )
include_directories( ${QRANSAC_SD_PRIM_SHAPES_LIB_SOURCE_DIR} )
include_directories( ${QRANSAC_SD_PRIM_SHAPES_LIB_SOURCE_DIR}/GfxTL )

endif()
