cmake_minimum_required(VERSION 2.8)

# Eliminate a warning when building in Windows that relates
# to static linking of Qt executables to qtmain.lib.
# This policy was introduced in CMake version 2.8.11.
# CMake version 2.8.11.2 warns when the policy is not set
# and uses OLD behavior.
if(POLICY CMP0020)
  cmake_policy(SET CMP0020 OLD)
endif()

option( INSTALL_QEDL_PLUGIN "Check to install qEDL plugin" OFF )

if (INSTALL_QEDL_PLUGIN)

#CloudCompare 'EDL' plugin
project( QEDL_PLUGIN )

set( CC_SHADER_FOLDER EDL )
include( ../CMakePluginTpl.cmake )

endif()
