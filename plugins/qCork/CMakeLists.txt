cmake_minimum_required(VERSION 2.8)

# Eliminate a warning when building in Windows that relates
# to static linking of Qt executables to qtmain.lib.
# This policy was introduced in CMake version 2.8.11.
# CMake version 2.8.11.2 warns when the policy is not set
# and uses OLD behavior.
if(POLICY CMP0020)
  cmake_policy(SET CMP0020 OLD)
endif()

option( INSTALL_QCORK_PLUGIN "Check to install qCORK plugin" OFF )

if (INSTALL_QCORK_PLUGIN)

#CloudCompare Mesh CSG operations plugin (based on the Cork library)
project( QCORK_PLUGIN )

# libfreenect is needed to build library
include( CorkSupport.cmake )

include( ../CMakePluginTpl.cmake )

target_link_cork( ${PROJECT_NAME} )

endif()
