cmake_minimum_required(VERSION 2.8)

option( INSTALL_QHPR_PLUGIN "Check to install qHPR plugin" OFF )

if (INSTALL_QHPR_PLUGIN)

#qHull is shared by multiple plugins
if (NOT ${OPTION_USE_QUICKHULL})

	message(FATAL_ERROR "The qHPR plugin relies on qHull! Enable OPTION_USE_QUICKHULL first")

else()

	#CloudCompare 'HPR' plugin
	project( QHPR_PLUGIN_DLL )

	include( ../CMakePluginTpl.cmake )

	target_link_libraries( ${PROJECT_NAME} qhull )
	include_directories( ${QHULL_LIB_INCLUDE_DIR} )

endif()

endif()
