cmake_minimum_required(VERSION 2.8)

# Eliminate a warning when building in Windows that relates
# to static linking of Qt executables to qtmain.lib.
# This policy was introduced in CMake version 2.8.11.
# CMake version 2.8.11.2 warns when the policy is not set
# and uses OLD behavior.
if(POLICY CMP0020)
  cmake_policy(SET CMP0020 OLD)
endif()

option( INSTALL_QSSAO_PLUGIN "Check to install qSSAO plugin" OFF )

if (INSTALL_QSSAO_PLUGIN)

add_subdirectory (Randomkit)

#CloudCompare 'SSAO' plugin
project( QSSAO_PLUGIN )

set( CC_SHADER_FOLDER SSAO )
include( ../CMakePluginTpl.cmake )

target_link_libraries( ${PROJECT_NAME} QSSAO_RANDOMKIT_LIB )
include_directories( ${QSSAO_RANDOMKIT_LIB_SOURCE_DIR} )

endif()
