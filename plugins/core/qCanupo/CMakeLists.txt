cmake_minimum_required(VERSION 2.8)

option( INSTALL_QCANUPO_PLUGIN "Check to install qCanupo plugin" OFF )

if (INSTALL_QCANUPO_PLUGIN)

#CloudCompare 'Canupo' plugin
project( QCANUPO_PLUGIN_DLL )

set( DLIB_ROOT "" CACHE PATH "DLib library root directory" )
if ( NOT DLIB_ROOT )
	message( SEND_ERROR "No DLib library root specified (DLIB_ROOT)" )
else()
	include_directories( ${DLIB_ROOT} )
endif()

include( ../../CMakePluginTpl.cmake )

option( COMPILE_QCANUPO_WITH_PRIVATE_PARTS "Check to compile qCanupo plugin with private methods (CNRS/UEB only)" OFF )
if ( COMPILE_QCANUPO_WITH_PRIVATE_PARTS )
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS COMPILE_PRIVATE_CANUPO )
endif()

endif()
