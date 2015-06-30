cmake_minimum_required(VERSION 2.8)

# Windows only for now
if (WIN32)

option( INSTALL_QCSV_MATRIX_IO_PLUGIN "Check to install qCSVMatrix I/O plugin" OFF )

if (INSTALL_QCSV_MATRIX_IO_PLUGIN)

#CloudCompare CSV Matrix I/O plugin
project( QCSV_MATRIX_IO_PLUGIN )

set( CC_IS_IO_PLUGIN 1 )

include( ../CMakePluginTpl.cmake )

endif()

endif()