cmake_minimum_required(VERSION 2.8)

# Eliminate a warning when building in Windows that relates
# to static linking of Qt executables to qtmain.lib.
# This policy was introduced in CMake version 2.8.11.
# CMake version 2.8.11.2 warns when the policy is not set
# and uses OLD behavior.
if(POLICY CMP0020)
  cmake_policy(SET CMP0020 OLD)
endif()

#REPLACE ALL 'DUMMY' OCCURENCES BY YOUR PLUGIN NAME
#AND ADAPT THE CODE BELOW TO YOUR OWN NEEDS!

option( INSTALL_QDUMMY_PLUGIN "Check to install qDUMMY plugin" OFF )

if (INSTALL_QDUMMY_PLUGIN)

#CloudCompare 'DUMMY' plugin
project( QDUMMY_PLUGIN )

#load necessary libraries (see qPCV for an example)
#add_subdirectory (LIB1)

#if the plugin is an 'OpenGL filter', uncomment the line below
#set( CC_OPENGL_FILTER ON BOOL)
include( ../CMakePluginTpl.cmake )

#set dependencies to necessary libraries (see qPCV for an example)
#target_link_libraries( ${PROJECT_NAME} LIB1 )
#include_directories( ${LIB1_INCLUDE_DIR} )

endif()
