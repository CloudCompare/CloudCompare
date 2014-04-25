cmake_minimum_required(VERSION 2.8)

# Eliminate a warning when building in Windows that relates
# to static linking of Qt executables to qtmain.lib.
# This policy was introduced in CMake version 2.8.11.
# CMake version 2.8.11.2 warns when the policy is not set
# and uses OLD behavior.
if(POLICY CMP0020)
  cmake_policy(SET CMP0020 OLD)
endif()

option( INSTALL_QHPR_PLUGIN "Check to install qHPR plugin" OFF )

if (INSTALL_QHPR_PLUGIN)

#CloudCompare 'HPR' plugin
project( QHPR_PLUGIN )

include( ../CMakePluginTpl.cmake )

add_subdirectory (qhull/src2012.1)

target_link_libraries( ${PROJECT_NAME} qhull )
include_directories( ${QHULL_LIB_INCLUDE_DIR} )

endif()
