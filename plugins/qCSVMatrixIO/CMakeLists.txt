cmake_minimum_required(VERSION 2.8)

# Eliminate a warning when building in Windows that relates
# to static linking of Qt executables to qtmain.lib.
# This policy was introduced in CMake version 2.8.11.
# CMake version 2.8.11.2 warns when the policy is not set
# and uses OLD behavior.
if(POLICY CMP0020)
  cmake_policy(SET CMP0020 OLD)
endif()

# Windows only for now
if (WIN32)

option( INSTALL_QCSV_MATRIX_IO_PLUGIN "Check to install qCSVMatrix I/O plugin" ON )

if (INSTALL_QCSV_MATRIX_IO_PLUGIN)

#CloudCompare CSV Matrix I/O plugin
project( QCSV_MATRIX_IO_PLUGIN )

set( CC_IS_IO_PLUGIN 1 )

include( ../CMakePluginTpl.cmake )

endif()

endif()