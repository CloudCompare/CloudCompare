cmake_minimum_required(VERSION 3.0)

option( PLUGIN_STANDARD_QPCV "Check to install qPCV plugin" OFF )

# CloudCompare 'PCV' plugin
if (PLUGIN_STANDARD_QPCV)
    add_subdirectory (PCV)

    project( QPCV_PLUGIN )
    
	include( CMakePolicies NO_POLICY_SCOPE )

	find_package( OpenGL REQUIRED )
	if( NOT OPENGL_FOUND )
		message( SEND_ERROR "OpenGL required, but not found with 'find_package()'" )
	endif()

	include( ../../../CMakePluginTpl.cmake )
    
	include_directories( ${OpenGL_INCLUDE_DIR} )
    include_directories( ${PCV_LIB_SOURCE_DIR} )
    
    target_link_libraries( ${PROJECT_NAME} PCV_LIB )
	target_link_libraries( ${PROJECT_NAME} ${OPENGL_LIBRARIES} )
endif()
